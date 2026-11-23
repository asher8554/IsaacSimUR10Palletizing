# UR10을 목표 자세로 움직인다. RMPflow(장애물을 피하는 반응형 제어)와 관절 직접 제어 두 가지를 쓴다
"""명령 하나(MotionCommand)는 "흡착컵 끝을 이 자세로"라는 뜻이다. 매 물리 스텝마다 다음 일을 한다.

    목표(컵 끝 자세) → tool0 자세로 환산 → RMPflow가 다음 관절 목표 계산 → 관절 구동기에 전달

NVIDIA 예제 isaacsim.robot_motion.examples의 ur10_palletizing.py(Apache-2.0)를 줄여 옮긴 것이다.
"""

from dataclasses import dataclass
from enum import Enum, auto

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.core.experimental.objects import Capsule, Sphere
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.experimental.utils.impl.transform import quaternion_conjugate, quaternion_multiplication
from isaacsim.robot_motion.cumotion import CumotionWorldInterface, RmpFlowController, load_cumotion_supported_robot
from isaacsim.robot_motion.schema import apply_motion_planning_api
from scipy.spatial.transform import Rotation

from palletizing.common import make_T, matrix_to_quat, quat_to_matrix
from palletizing.gripper import TIP_OFFSET, SuctionGripper

HOME_JOINTS = np.array([-1.57, -1.57, -1.57, -1.57, 1.57, 0.0])  # 홈 자세(rad)

# ---------------------------------------------------------------------------
# 명령 형식
# ---------------------------------------------------------------------------


@dataclass
class Approach:
    """마지막 접근을 곧게 만드는 깔때기. direction(길이 포함) 방향으로만 목표에 다가가게 한다."""

    direction: np.ndarray
    std_dev: float


@dataclass
class MotionCommand:
    """흡착컵 끝의 목표 자세. joint_target을 주면 관절값으로 직접 간다."""

    p: np.ndarray  # 위치
    q: np.ndarray  # 쿼터니언 (w,x,y,z)
    approach: Approach | None = None
    joint_target: np.ndarray | None = None

    def target_T(self) -> np.ndarray:
        return make_T(self.p, quat_to_matrix(self.q))


def shift_for_approach(target_T: np.ndarray, tip_T: np.ndarray, approach: Approach) -> np.ndarray:
    """컵 끝이 접근선에서 벗어난 만큼 목표를 접근 방향 반대로 물린다.

    접근선(목표를 지나 direction 방향인 직선)에서 멀면 목표가 direction만큼 뒤로 물러나 있고,
    접근선 위에 올라올수록 원래 목표로 돌아온다. 그래서 "먼저 위로 맞춘 뒤 곧게 내려가는" 경로가 된다.
    """
    v = tip_T[:3, 3] - target_T[:3, 3]
    unit = approach.direction / np.linalg.norm(approach.direction)
    off_line = np.linalg.norm(v - np.dot(v, unit) * unit)  # 접근선까지의 수직 거리
    off_line += 0.5 * np.linalg.norm(target_T[:3, :3] - tip_T[:3, :3]) / 3  # 자세가 틀어진 만큼도 더한다
    alpha = 1.0 - np.exp(-0.5 * off_line * off_line / (approach.std_dev * approach.std_dev))
    return target_T[:3, 3] - alpha * approach.direction


def make_approach(direction, distance: float, max_length: float, std_dev: float, lookahead: float = 0.08):
    """목표까지 남은 거리에 맞춰 깔때기 길이를 정한다. 다 왔으면(None) 깔때기를 끈다."""
    length = min(max_length, max(0.0, distance - lookahead))
    return Approach(length * np.asarray(direction), std_dev) if length > 1.0e-6 else None


# ---------------------------------------------------------------------------
# 가상 장애물. 화면에는 안 보이고 경로 계산에만 쓰인다.
# ---------------------------------------------------------------------------

OBSTACLE_ROOT = "/World/PlanningObstacles"
FLIP_STATION_OBSTACLES = [f"{OBSTACLE_ROOT}/FlipStationSphere"]  # 뒤집기 장치 위를 덮는 공
NAVIGATION_OBSTACLES = [
    f"{OBSTACLE_ROOT}/NavigationDome",  # 로봇 아래를 채우는 큰 공. 팔이 아래로 처지지 않게 한다
    f"{OBSTACLE_ROOT}/NavigationBarrier",  # 컨베이어와 팔레트 사이의 칸막이
    f"{OBSTACLE_ROOT}/NavigationFlipStation",  # 뒤집기 장치 기둥
]
ALL_OBSTACLES = FLIP_STATION_OBSTACLES + NAVIGATION_OBSTACLES


def create_planning_obstacles() -> None:
    """경로 계산용 장애물을 만든다. 재생 전에 한 번 부른다."""
    stage_utils.define_prim(OBSTACLE_ROOT, "Xform")
    Sphere(FLIP_STATION_OBSTACLES[0], positions=[0.73, 0.76, -0.13], radii=0.2)
    Sphere(NAVIGATION_OBSTACLES[0], positions=[-0.031, -0.018, -1.086], radii=1.1)

    az = np.array([1.0, 0.0, -0.3]) / np.linalg.norm([1.0, 0.0, -0.3])
    ax = np.array([0.0, 1.0, 0.0])
    Capsule(
        NAVIGATION_OBSTACLES[1],
        positions=[0.471, 0.276, -0.563],
        orientations=matrix_to_quat(np.column_stack([ax, np.cross(az, ax), az])),
        radii=0.5,
        heights=0.9,
    )
    Capsule(NAVIGATION_OBSTACLES[2], positions=[0.766, 0.755, -0.5], radii=0.5, heights=0.5)

    stage = stage_utils.get_current_stage(backend="usd")
    for path in ALL_OBSTACLES:
        apply_motion_planning_api(stage.GetPrimAtPath(path), enabled=False)  # 일단 꺼 둔다
    XformPrim(ALL_OBSTACLES).set_visibilities(False)


# ---------------------------------------------------------------------------
# 제어기 두 개
# ---------------------------------------------------------------------------


class _Mode(Enum):
    RMPFLOW = auto()
    JOINT_TARGET = auto()


class ToolSpaceRmpFlowController(mg.BaseController):
    """RMPflow를 감싼 것. 목표가 갑자기 멀리 바뀌면 내부 상태를 초기화한다."""

    def __init__(self, rmp_flow, world_binding, articulation, tool_frame, reset_position_delta, reset_rotation_delta):
        self._rmp_flow = rmp_flow
        self._world_binding = world_binding
        self._articulation = articulation
        self._tool_frame = tool_frame
        self._reset_position_delta = reset_position_delta
        self._reset_rotation_delta = reset_rotation_delta
        self._reset_needed = True
        self._last_p = None
        self._last_R = None

    def request_reset(self) -> None:
        self._reset_needed = True

    def needs_reset(self, setpoint) -> bool:
        return self._reset_needed or self._target_jumped(setpoint)

    def reset(self, estimated_state, setpoint_state, t, **kwargs) -> bool:
        self._sync_world()
        if not self._rmp_flow.reset(estimated_state, setpoint_state, t, **kwargs):
            self._reset_needed = True
            return False
        self._reset_needed = False
        self._remember(setpoint_state)
        return True

    def forward(self, estimated_state, setpoint_state, t, **kwargs):
        if self._target_jumped(setpoint_state):
            self._reset_needed = True
        if self._reset_needed and not self.reset(estimated_state, setpoint_state, t, **kwargs):
            raise RuntimeError("RMPflow 초기화 실패")
        self._sync_world()
        self._remember(setpoint_state)
        return self._rmp_flow.forward(estimated_state, setpoint_state, t, **kwargs)

    def _sync_world(self) -> None:
        """로봇 베이스 위치와 장애물 위치를 경로 계산기에 알려 준다."""
        interface = self._world_binding.get_world_interface()
        interface.update_world_to_robot_root_transforms(self._articulation.get_world_poses())
        self._world_binding.synchronize_transforms()

    def _remember(self, setpoint) -> None:
        pose = self._tool_pose(setpoint)
        if pose is not None:
            self._last_p, self._last_R = pose

    def _target_jumped(self, setpoint) -> bool:
        pose = self._tool_pose(setpoint)
        if pose is None:
            return False
        if self._last_p is None:
            return True
        p_delta = np.linalg.norm(pose[0] - self._last_p)
        r_delta = np.linalg.norm(pose[1] - self._last_R) / 3
        return bool(p_delta > self._reset_position_delta or r_delta > self._reset_rotation_delta)

    def _tool_pose(self, setpoint):
        if setpoint is None or setpoint.sites is None:
            return None
        sites = setpoint.sites
        if self._tool_frame not in sites.position_names or self._tool_frame not in sites.orientation_names:
            return None
        p = np.asarray(sites.positions.numpy()[sites.position_names.index(self._tool_frame)], dtype=float)
        q = np.asarray(sites.orientations.numpy()[sites.orientation_names.index(self._tool_frame)], dtype=float)
        return p.copy(), quat_to_matrix(q)


class DirectJointTargetController(mg.BaseController):
    """관절 목표로 곧장 간다. 한 스텝에 delta_q_max(rad)까지만 움직여 속도를 제한한다."""

    def __init__(self, robot_joint_space, controlled_joint_names, delta_q_max):
        self._robot_joint_space = robot_joint_space
        self._names = controlled_joint_names
        self._delta_q_max = delta_q_max

    def reset(self, estimated_state, setpoint_state, t, **kwargs) -> bool:
        return self._positions(estimated_state.joints) is not None

    def forward(self, estimated_state, setpoint_state, t, **kwargs):
        if setpoint_state is None:
            return None
        current = self._positions(estimated_state.joints)
        target = self._positions(setpoint_state.joints)
        if current is None or target is None:
            return None
        delta = target - current
        biggest = float(np.max(np.abs(delta)))
        if biggest > self._delta_q_max:
            delta = delta * (self._delta_q_max / biggest)
        dt = max(float(kwargs.get("dt", 1.0 / 60.0)), 1.0e-6)
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                positions=(self._names, wp.array((current + delta).tolist(), dtype=wp.float32)),
                velocities=(self._names, wp.array((delta / dt).tolist(), dtype=wp.float32)),
            )
        )

    def _positions(self, joints):
        if joints is None or joints.positions is None or not set(self._names).issubset(joints.position_names):
            return None
        values = joints.positions.numpy()
        return np.asarray([values[joints.position_names.index(name)] for name in self._names], dtype=float)


# ---------------------------------------------------------------------------
# 로봇
# ---------------------------------------------------------------------------


class Ur10:
    """UR10 + 흡착 그리퍼 + 모션 제어기."""

    # USD의 ee_link 좌표축과 cuMotion의 tool0 좌표축이 다르게 놓여 있어 보정 회전이 필요하다.
    _EE_LINK_FROM_WRIST = Rotation.from_euler("z", np.pi / 2.0).as_quat()[[3, 0, 1, 2]]
    _TOOL0_FROM_WRIST = Rotation.from_euler("x", -np.pi / 2.0).as_quat()[[3, 0, 1, 2]]

    def __init__(self, robot_path: str):
        self.path = robot_path
        self.gripper = SuctionGripper(robot_path)
        self.articulation: Articulation | None = None
        self._command: MotionCommand | None = None
        self._home_tip_T: np.ndarray | None = None
        self._time = 0.0
        self._obstacle_enabled: dict[str, bool] = {}

    # ---- 준비 -------------------------------------------------------------

    def setup(self) -> None:
        """재생 전에 한 번. 에셋이 다 열리고 create_planning_obstacles()를 부른 뒤여야 한다."""
        self.articulation = Articulation(self.path)
        self.articulation.set_link_enabled_gravities([False])  # 팔이 자기 무게로 처지지 않게
        self.articulation.set_default_state(dof_positions=[HOME_JOINTS])
        self._build_world_binding()
        self._build_controllers()

    def start(self) -> None:
        """재생이 시작된 뒤 한 번. 그리퍼를 준비하고 홈 자세로 보낸다."""
        self.gripper.initialize()
        self.reset()

    def reset(self) -> None:
        zeros = np.zeros_like(HOME_JOINTS)
        self.articulation.set_dof_positions(HOME_JOINTS.tolist())
        self.articulation.set_dof_position_targets(HOME_JOINTS.tolist())
        self.articulation.set_dof_velocities(zeros.tolist())
        self.articulation.set_dof_velocity_targets(zeros.tolist())
        self._time = 0.0
        self._home_tip_T = None
        self._rmp.request_reset()
        self._selector.set_next_controller(_Mode.RMPFLOW)
        self._command = None
        self.gripper.open()

    def _build_world_binding(self) -> None:
        """장애물 prim들을 경로 계산기(cuMotion)의 세계에 연결한다."""
        self._world_binding = mg.WorldBinding(
            world_interface=CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=ALL_OBSTACLES,
            tracked_collision_api=mg.TrackableApi.MOTION_GENERATION_COLLISION,
        )
        self._world_binding.initialize()
        self._obstacle_enabled = {path: False for path in ALL_OBSTACLES}
        interface = self._world_binding.get_world_interface()
        interface.update_world_to_robot_root_transforms(poses=self.articulation.get_world_poses())
        self._world_binding.synchronize_transforms()

    def _build_controllers(self) -> None:
        names = self.articulation.dof_names
        cumotion_robot = load_cumotion_supported_robot("ur10")  # UR10용 RMPflow 설정이 Isaac Sim에 들어 있다
        self._sites = cumotion_robot.robot_description.tool_frame_names()
        self._tool_frame = self._sites[0]  # "tool0"
        rmp_flow = RmpFlowController(
            cumotion_robot=cumotion_robot,
            cumotion_world_interface=self._world_binding.get_world_interface(),
            robot_joint_space=names,
            robot_site_space=self._sites,
            tool_frame=self._tool_frame,
        )
        rmp_flow.get_rmp_flow_config().set_param("cspace_target_rmp/metric_scalar", 1.0)
        self._rmp = ToolSpaceRmpFlowController(
            rmp_flow, self._world_binding, self.articulation, self._tool_frame,
            reset_position_delta=0.12, reset_rotation_delta=0.2,
        )  # fmt: skip
        joint_controller = DirectJointTargetController(names, names[:6], delta_q_max=0.05)
        self._selector = mg.SelectableController(
            controller_options={_Mode.RMPFLOW: self._rmp, _Mode.JOINT_TARGET: joint_controller},
            initial_controller_selection=_Mode.RMPFLOW,
        )

    # ---- 읽기 -------------------------------------------------------------

    def tip_T(self) -> np.ndarray:
        """지금 흡착컵 끝의 월드 자세(4x4)."""
        return self.gripper.tip_T()

    @property
    def home_tip_T(self) -> np.ndarray:
        """홈 자세일 때의 컵 끝 자세. 첫 step()에서 기록한다."""
        return self.tip_T() if self._home_tip_T is None else self._home_tip_T

    # ---- 명령 -------------------------------------------------------------

    def send(self, command: MotionCommand | None) -> None:
        """다음 step()부터 따를 명령을 바꾼다. None이면 그 자리에 머문다."""
        self._command = command

    def step(self, dt: float) -> None:
        """매 물리 스텝마다 부른다."""
        if self._home_tip_T is None:
            self._home_tip_T = self.tip_T()
        command = self._command
        if command is None:
            return
        if command.joint_target is not None:
            self._select(_Mode.JOINT_TARGET)
            self._run_controller(self._joint_setpoint(command.joint_target), dt)
            return
        self._select(_Mode.RMPFLOW)
        self._run_controller(self._tool_setpoint(command), dt)

    def set_obstacles_enabled(self, paths: list[str], enabled: bool) -> None:
        changed = [path for path in paths if self._obstacle_enabled.get(path) != enabled]
        if not changed:
            return
        interface = self._world_binding.get_world_interface()
        interface.update_obstacle_enables(changed, wp.array([enabled] * len(changed), dtype=wp.bool))
        for path in changed:
            self._obstacle_enabled[path] = enabled

    # ---- 내부 -------------------------------------------------------------

    def _select(self, mode: _Mode) -> None:
        active = self._selector.get_active_controller_enum()
        self._selector.set_next_controller(mode)
        if active != mode and mode == _Mode.RMPFLOW:
            self._time = 0.0
            self._rmp.request_reset()

    def _run_controller(self, setpoint, dt: float) -> None:
        names = self.articulation.dof_names
        estimated = mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=names,
                positions=(names, self.articulation.get_dof_positions()),
                velocities=(names, self.articulation.get_dof_velocities()),
            )
        )
        using_rmp = self._selector.get_active_controller_enum() == _Mode.RMPFLOW
        if using_rmp and self._rmp.needs_reset(setpoint):
            self._time = 0.0
        desired = self._selector.forward(estimated, setpoint, self._time, dt=dt)
        if self._selector.get_active_controller_enum() == _Mode.RMPFLOW:
            self._time += dt
        if desired is None or desired.joints is None:
            return
        joints = desired.joints
        if joints.positions is not None:
            self.articulation.set_dof_position_targets(joints.positions, dof_indices=_indices(joints.position_indices))
        if joints.velocities is not None:
            self.articulation.set_dof_velocity_targets(joints.velocities, dof_indices=_indices(joints.velocity_indices))

    def _tool_setpoint(self, command: MotionCommand):
        """컵 끝 목표 → tool0 목표. 접근 깔때기도 여기서 적용한다."""
        p_tip = command.p
        if command.approach is not None:
            p_tip = shift_for_approach(command.target_T(), self.tip_T(), command.approach)
        p_tool = p_tip - quat_to_matrix(command.q) @ TIP_OFFSET  # 컵 길이만큼 뒤로
        q_tool = self._tool_orientation(command.q)
        return mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=self._sites,
                positions=([self._tool_frame], wp.array([p_tool.tolist()], dtype=wp.float32)),
                orientations=([self._tool_frame], wp.array([q_tool.tolist()], dtype=wp.float32)),
            )
        )

    def _joint_setpoint(self, joint_target):
        names = self.articulation.dof_names
        values = np.asarray(joint_target, dtype=float)[:6].tolist()
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=names, positions=(names[:6], wp.array(values, dtype=wp.float32))
            )
        )

    @classmethod
    def _tool_orientation(cls, ee_orientation) -> np.ndarray:
        """ee_link 기준 자세를 tool0 기준 자세로 바꾼다."""
        wrist_from_ee = quaternion_conjugate(wp.array([cls._EE_LINK_FROM_WRIST], dtype=wp.float32))
        ee = wp.array([np.asarray(ee_orientation).tolist()], dtype=wp.float32)
        tool0_from_wrist = wp.array([cls._TOOL0_FROM_WRIST], dtype=wp.float32)
        q = quaternion_multiplication(quaternion_multiplication(ee, wrist_from_ee), tool0_from_wrist).numpy()[0]
        return q / np.linalg.norm(q)


def _indices(indices):
    if indices is None:
        return None
    if hasattr(indices, "numpy"):
        return indices.numpy().flatten().astype(np.int64).tolist()
    return np.asarray(indices, dtype=np.int64).flatten().tolist()
