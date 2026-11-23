# 집기·뒤집기·놓기를 이루는 작은 동작 조각들과, 그 조각들이 주고받는 관측·명령 형식
"""한 스텝의 흐름:  관측(Observation) → 동작 조각.step() → 명령(Command) → 셀이 실행

동작 조각은 시뮬레이터를 직접 만지지 않는다. 관측만 보고 명령만 내놓는다.
그래서 조각을 이어 붙이는 순서만 바꾸면 다른 작업을 만들 수 있다.
"""

from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

from palletizing.common import matrix_to_quat, transforms_close
from palletizing.motion import Approach, MotionCommand, make_approach


class Grip(Enum):
    HOLD = auto()  # 그대로
    OPEN = auto()  # 놓기
    CLOSE = auto()  # 붙이기


@dataclass(frozen=True)
class Observation:
    """한 스텝에 본 것 전부."""

    time: float
    tip_T: np.ndarray  # 흡착컵 끝 자세
    home_tip_T: np.ndarray  # 홈 자세일 때 컵 끝 자세
    home_joints: np.ndarray
    gripper_closed: bool
    has_active_bin: bool  # 집을 박스가 있는가
    bin_attached: bool
    bin_needs_flip: bool
    bin_grasp_T: np.ndarray | None  # 집을 자세
    bin_position: np.ndarray | None
    stack_target: np.ndarray | None  # 이번 박스를 놓을 자리
    bin_under_position: np.ndarray | None  # 그 자리 아래층 박스의 실제 위치
    bin_at_stack_target: bool  # 박스가 놓을 자리에 와 있는가
    last_bin: bool  # 이번 박스가 마지막인가


@dataclass(frozen=True)
class Command:
    """한 스텝에 할 일."""

    motion: MotionCommand | None = None
    retain_motion: bool = False  # motion이 None일 때 직전 명령을 유지할지
    gripper: Grip = Grip.HOLD
    navigation_obstacles: bool = False  # 큰 이동용 가상 장애물을 쓸지
    flip_station_obstacle: bool = False  # 뒤집기 장치 위 장애물을 쓸지
    complete_bin: bool = False  # 이 박스 작업이 끝났음을 알림
    conveyor_running: bool | None = None  # None이면 그대로


def flip_x_if_opposite(tip_R: np.ndarray, target_R: np.ndarray) -> np.ndarray:
    """목표 자세가 지금 자세와 정반대(180도)면 같은 접근 방향의 다른 해를 고른다. 손목이 크게 도는 것을 막는다."""
    if np.dot(target_R[:3, 0], tip_R[:3, 0]) < -0.9:
        return target_R @ np.diag([1, -1, -1])
    return target_R


# ---------------------------------------------------------------------------
# 동작 조각. enter()는 시작할 때 한 번, step()은 매 스텝 불린다. step()은 (끝났는가, 명령)을 돌려준다.
# ---------------------------------------------------------------------------


class Task:
    def enter(self, obs: Observation) -> None:
        pass

    def step(self, obs: Observation) -> tuple[bool, Command]:
        return True, Command()


class Wait(Task):
    """정해진 시간만큼 기다린다. 끝날 때 컨베이어를 세울 수 있다."""

    def __init__(self, seconds: float, pause_conveyor_on_done: bool = False):
        self._seconds = seconds
        self._pause = pause_conveyor_on_done

    def enter(self, obs):
        self._started = obs.time

    def step(self, obs):
        done = obs.time - self._started >= self._seconds
        return done, Command(retain_motion=True, conveyor_running=False if done and self._pause else None)


class Move(Task):
    """목표 자세에 닿을 때까지 움직인다. 하위 클래스가 get_motion()으로 목표를 정한다."""

    navigation_obstacles = False
    flip_station_obstacle = False

    def __init__(self, p_thresh: float, R_thresh: float):
        self.p_thresh = p_thresh
        self.R_thresh = R_thresh

    def get_motion(self, obs: Observation) -> MotionCommand:
        raise NotImplementedError

    def step(self, obs):
        motion = self.get_motion(obs)
        done = transforms_close(motion.target_T(), obs.tip_T, self.p_thresh, self.R_thresh)
        return done, Command(motion, navigation_obstacles=self.navigation_obstacles, flip_station_obstacle=self.flip_station_obstacle)  # fmt: skip


class ReachToPick(Move):
    """박스의 집을 자세로 다가간다. 박스가 조금 움직여도 매 스텝 다시 읽은 자세를 따라간다."""

    navigation_obstacles = True
    flip_station_obstacle = True

    def __init__(self):
        super().__init__(p_thresh=0.005, R_thresh=2.0)

    def get_motion(self, obs):
        R, p = obs.bin_grasp_T[:3, :3], obs.bin_grasp_T[:3, 3]
        approach_dir = R[:, 0]
        R = flip_x_if_opposite(obs.tip_T[:3, :3], R)
        distance = np.linalg.norm(p - obs.tip_T[:3, 3])
        if obs.bin_needs_flip:
            # 입구가 위인 박스는 벽 안쪽으로 들어가야 한다. 입구 바로 위에 올 때까지 긴 깔때기(30 cm)를 유지한다.
            v = obs.tip_T[:3, 3] - p
            sideways = np.linalg.norm(v - np.dot(v, approach_dir) * approach_dir)
            approach = Approach(0.3 * approach_dir, 0.005) if sideways > 0.03 else make_approach(approach_dir, distance, 0.3, 0.005)  # fmt: skip
        else:
            approach = make_approach(approach_dir, distance, 0.1, 0.005)
        return MotionCommand(p, matrix_to_quat(R), approach)


class ReachToPlace(Move):
    """팔레트의 놓을 자리로 내려간다. 아래층 박스가 있으면 그 실제 위치에 맞춰 살짝 보정한다."""

    navigation_obstacles = True

    def __init__(self):
        super().__init__(p_thresh=0.02, R_thresh=2.0)
        down = np.array([0.0, 0.0, -1.0])
        z_axis = np.array([0.0, -1.0, 0.0])
        self._R = np.column_stack([down, np.cross(z_axis, down), z_axis])  # 컵이 아래를 보는 자세

    def enter(self, obs):
        self._base_p = obs.stack_target.copy()
        self._p = self._base_p.copy()

    def get_motion(self, obs):
        if obs.bin_under_position is not None and obs.bin_position is not None:
            xy_error = obs.bin_under_position[:2] - obs.bin_position[:2]
            if np.linalg.norm(xy_error) < 0.02:
                self._p[:2] = self._base_p[:2] + 0.1 * xy_error
        self._R = flip_x_if_opposite(obs.tip_T[:3, :3], self._R)
        distance = np.linalg.norm(self._p - obs.tip_T[:3, 3])
        return MotionCommand(self._p, matrix_to_quat(self._R), make_approach([0.0, 0.0, -1.0], distance, 0.35, 0.005))


class CloseGripper(Task):
    """흡착을 켜고 실제로 붙을 때까지 기다린다."""

    def enter(self, obs):
        print("<흡착 켜기>")

    def step(self, obs):
        return obs.gripper_closed, Command(retain_motion=True, gripper=Grip.CLOSE)


class OpenGripper(Task):
    def enter(self, obs):
        print("<흡착 끄기>")

    def step(self, obs):
        return True, Command(retain_motion=True, gripper=Grip.OPEN)


class LiftToClearance(Move):
    """지금 자세 그대로 수직으로 height만큼 올라간다(도착할 때까지)."""

    def __init__(self, height: float):
        super().__init__(p_thresh=0.02, R_thresh=2.0)
        self._height = height

    def enter(self, obs):
        self._motion = MotionCommand(obs.tip_T[:3, 3] + [0.0, 0.0, self._height], matrix_to_quat(obs.tip_T[:3, :3]))

    def get_motion(self, obs):
        return self._motion


class TimedLift(Task):
    """수직으로 올라가되 도착을 기다리지 않고 seconds 뒤에 다음 동작으로 넘어간다."""

    def __init__(self, height: float, seconds: float):
        self._height = height
        self._seconds = seconds

    def enter(self, obs):
        self._started = obs.time
        self._motion = MotionCommand(obs.tip_T[:3, 3] + [0.0, 0.0, self._height], matrix_to_quat(obs.tip_T[:3, :3]))

    def step(self, obs):
        return obs.time - self._started >= self._seconds, Command(self._motion)


class OrientToHome(Move):
    """관절 제어로 홈 자세에 돌아간다."""

    def __init__(self):
        super().__init__(p_thresh=0.005, R_thresh=0.1)

    def get_motion(self, obs):
        T = obs.home_tip_T
        return MotionCommand(T[:3, 3], matrix_to_quat(T[:3, :3]), joint_target=obs.home_joints)


class MoveToFlipStation(Move):
    """박스를 든 채 뒤집기 장치의 정해진 자세로 간다. 비스듬히 기울여 걸쳐 놓는 자세다."""

    _APPROACH_DIR = np.array([0.5, -0.3, -0.75]) / np.linalg.norm([0.5, -0.3, -0.75])

    def __init__(self):
        super().__init__(p_thresh=0.065, R_thresh=2.0)
        self._p = np.array([0.7916634, 0.73902607, -0.02897218])
        self._q = np.array([0.52239186, 0.6296602, -0.5042411, 0.27636158])

    def get_motion(self, obs):
        distance = np.linalg.norm(self._p - obs.tip_T[:3, 3])
        return MotionCommand(self._p, self._q, make_approach(self._APPROACH_DIR, distance, 0.4, 0.05))


class ReleaseFlipStationBin(Task):
    """박스를 놓은 뒤 컵을 박스 밖으로 빼면서 로봇 쪽으로 물러난다."""

    _TOWARD_BASE = 0.2 * np.array([-1.0, -0.3, 0.0]) / np.linalg.norm([-1.0, -0.3, 0.0])

    def enter(self, obs):
        p = obs.tip_T[:3, 3] - 0.3 * obs.tip_T[:3, 0] + self._TOWARD_BASE  # 컵 축 반대로 30 cm
        self._motion = MotionCommand(p, matrix_to_quat(np.diag([1.0, -1.0, -1.0])), Approach(self._TOWARD_BASE, 0.1))

    def step(self, obs):
        return np.linalg.norm(self._motion.p - obs.tip_T[:3, 3]) < 0.15, Command(self._motion)


# ---------------------------------------------------------------------------
# 조각을 이어 붙인 작업 순서
# ---------------------------------------------------------------------------


def pick_sequence(obs: Observation, after_flip: bool = False) -> list[Task]:
    """컨베이어(또는 뒤집기 장치)에서 박스를 집어 든다."""
    return [
        # 박스가 구역에 들어오고 1초 뒤 컨베이어를 세운다. 뒤집기 뒤에는 컨베이어를 건드리지 않는다.
        Wait(1.0) if after_flip else Wait(1.0, pause_conveyor_on_done=True),
        ReachToPick(),
        Wait(1.0),
        CloseGripper(),
        # 뒤집을 박스는 짧게 들고 바로 이동, 쌓을 박스는 50 cm를 확실히 들어 올린다.
        TimedLift(0.3, 0.1) if obs.bin_needs_flip else LiftToClearance(0.5),
    ]


def flip_sequence() -> list[Task]:
    """박스를 뒤집기 장치에 걸쳐 놓는다. 장치의 경사 때문에 박스가 넘어가며 뒤집힌다."""
    return [OrientToHome(), MoveToFlipStation(), OpenGripper(), Wait(0.25), ReleaseFlipStationBin(), OrientToHome()]


def place_sequence() -> list[Task]:
    """팔레트의 다음 자리에 박스를 놓고 홈으로 돌아간다."""
    return [ReachToPlace(), Wait(0.5), OpenGripper(), TimedLift(0.1, 0.25), OrientToHome()]
