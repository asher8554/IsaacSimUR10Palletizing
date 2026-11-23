# 셀 전체의 상태를 읽어 관측을 만들고, 명령을 로봇·그리퍼·컨베이어에 실행한다
"""read()와 apply() 두 가지만 한다. 판단은 하지 않는다(판단은 tasks.py와 fsm.py의 몫)."""

import numpy as np

from palletizing.common import rigid_pose, transforms_close
from palletizing.motion import ALL_OBSTACLES, FLIP_STATION_OBSTACLES, NAVIGATION_OBSTACLES, HOME_JOINTS, Ur10
from palletizing.perception import BinState, estimate_grasp, find_active_bin
from palletizing.scene import ENV_PATH, set_conveyor_running
from palletizing.spawner import BinSpawner
from palletizing.tasks import Command, Grip, Observation

BIN_HEIGHT = 0.135  # 겹쳐 쌓을 때 한 층 높이
PALLET_TOP_Z = -0.59374  # 팔레트 윗면 높이


def stack_coordinates() -> list[np.ndarray]:
    """팔레트 위에 놓을 자리 9곳(3x3). 흡착컵 끝이 가야 할 위치다."""
    xs = np.array([1.00, 0.79, 0.58]) + 0.05  # 로봇에서 먼 열부터
    ys = [-0.62, -0.31, 0.0]
    z = PALLET_TOP_Z + BIN_HEIGHT / 2 + 0.0075  # 살짝 띄운 높이에서 놓는다
    return [np.array([x, y, z]) for y in ys for x in xs]


class PalletizingCell:
    def __init__(self, robot: Ur10, spawner: BinSpawner, max_bins: int | None = None, env_path: str = ENV_PATH):
        self.robot = robot
        self.spawner = spawner
        self.env_path = env_path
        self.targets = stack_coordinates()[:max_bins]
        self.bins: list[BinState] = []  # 지금까지 본 박스
        self.active: BinState | None = None  # 지금 다루는 박스
        self.stacked: list[BinState] = []  # 팔레트에 올린 박스
        self._observation: Observation | None = None
        self._conveyor_running: bool | None = None

    def reset(self) -> None:
        self.bins.clear()
        self.active = None
        self.stacked.clear()
        self._observation = None
        self._set_conveyor(True)
        self.robot.set_obstacles_enabled(ALL_OBSTACLES, False)

    # ---- 읽기 -------------------------------------------------------------

    def read(self, time: float) -> Observation:
        tip_T = self.robot.tip_T()
        self._update_active_bin(tip_T)

        target = self.targets[len(self.stacked)].copy() if len(self.stacked) < len(self.targets) else None
        under = self._bin_under(target) if target is not None else None
        active = self.active
        self._observation = Observation(
            time=time,
            tip_T=tip_T.copy(),
            home_tip_T=self.robot.home_tip_T.copy(),
            home_joints=HOME_JOINTS.copy(),
            gripper_closed=self.robot.gripper.is_closed(),
            has_active_bin=active is not None,
            bin_attached=bool(active and active.is_attached),
            bin_needs_flip=bool(active and active.needs_flip),
            bin_grasp_T=active.grasp_T.copy() if active else None,
            bin_position=rigid_pose(active.rigid)[0] if active else None,
            stack_target=target,
            bin_under_position=rigid_pose(under.rigid)[0] if under else None,
            bin_at_stack_target=self._active_at_target(target),
            last_bin=len(self.stacked) + 1 >= len(self.targets),
        )
        return self._observation

    def _update_active_bin(self, tip_T: np.ndarray) -> None:
        # (1) 다루는 박스가 없으면 작업 구역에서 새로 고른다.
        if self.active is None:
            for rigid in self.spawner.bins[len(self.bins) :]:
                self.bins.append(BinState(rigid))
            self.active = find_active_bin(self.bins)
        # (2) 박스가 없거나, 바닥으로 떨어졌거나, 이미 들어 올렸으면 컨베이어를 다시 돌린다.
        if self.active is None:
            self._set_conveyor(True)
            return
        if rigid_pose(self.active.rigid)[0][2] < -1.0:
            self.active = None
            self._set_conveyor(True)
            return
        if self.active.is_attached:
            self._set_conveyor(True)
        # (3) 집을 자세를 매 스텝 다시 계산한다. 들고 있을 때는 중력 방향 대신 컵 방향을 "위"의 기준으로 쓴다.
        up_vec = -tip_T[:3, 0] if self.active.is_attached else (0.0, 0.0, 1.0)
        estimate_grasp(self.active, up_vec)
        # (4) 붙었는가: 컵이 집을 자세 근처에 있고 그리퍼가 Closed 상태.
        self.active.is_attached = transforms_close(self.active.grasp_T, tip_T, 0.1, 1.0) and self.robot.gripper.is_closed()

    def _bin_under(self, target: np.ndarray) -> BinState | None:
        """놓을 자리 바로 아래에 이미 쌓인 박스."""
        for bin_state in reversed(self.stacked):
            if np.linalg.norm(rigid_pose(bin_state.rigid)[0][:2] - target[:2]) < 0.05:
                return bin_state
        return None

    def _active_at_target(self, target) -> bool:
        if self.active is None or target is None:
            return False
        p = rigid_pose(self.active.rigid)[0]
        return bool(np.linalg.norm(p[:2] - target[:2]) < 0.12 and abs(p[2] - target[2]) < 0.16)

    # ---- 실행 -------------------------------------------------------------

    def apply(self, command: Command, dt: float) -> None:
        obs = self._observation
        if command.motion is None:
            self.robot.set_obstacles_enabled(ALL_OBSTACLES, False)
            if not command.retain_motion:
                self.robot.send(None)
        else:
            self.robot.set_obstacles_enabled(
                NAVIGATION_OBSTACLES, command.navigation_obstacles and self._crossing_sides(command.motion.p, obs)
            )
            self.robot.set_obstacles_enabled(
                FLIP_STATION_OBSTACLES, command.flip_station_obstacle and not self._inside_bin(obs)
            )
            self.robot.send(command.motion)

        if command.gripper is Grip.OPEN:
            self.robot.gripper.open()
        elif command.gripper is Grip.CLOSE:
            self.robot.gripper.close()

        if command.conveyor_running is not None:
            self._set_conveyor(command.conveyor_running)
            if not command.conveyor_running and self.active is not None:
                self.active.rigid.set_velocities(linear_velocities=[np.zeros(3)])  # 박스도 그 자리에 세운다

        if command.complete_bin:
            self.stacked.append(self.active)
            self.active = None
        self.robot.step(dt)

    @staticmethod
    def _crossing_sides(target_p: np.ndarray, obs: Observation) -> bool:
        """컵과 목표가 컨베이어 쪽 / 팔레트 쪽으로 갈라져 있는가. 그럴 때만 큰 이동용 장애물을 켠다."""
        divider = np.array([0.6, 0.37, 0.0])  # 로봇에서 두 구역 사이로 뻗는 기준선
        tip = obs.tip_T[:3, 3] * [1.0, 1.0, 0.0]
        target = np.asarray(target_p, dtype=float) * [1.0, 1.0, 0.0]
        return bool(np.sign(np.cross(target, divider)[2]) * np.sign(np.cross(tip, divider)[2]) < 0.0)

    @staticmethod
    def _inside_bin(obs: Observation) -> bool:
        """컵이 이미 박스 바로 위(접근선 위)까지 왔는가. 그때는 뒤집기 장치 장애물을 꺼야 끝까지 내려갈 수 있다."""
        if obs.bin_grasp_T is None:
            return False
        axis = obs.bin_grasp_T[:3, 0]
        v = obs.tip_T[:3, 3] - obs.bin_grasp_T[:3, 3]
        along = v.dot(axis)
        sideways = np.linalg.norm(v - along * axis)
        return bool(along < 0.02 and axis.dot(obs.tip_T[:3, 0]) > 0.75 and sideways < 0.03)

    def _set_conveyor(self, running: bool) -> None:
        if running != self._conveyor_running:
            set_conveyor_running(running, self.env_path)
            self._conveyor_running = running
