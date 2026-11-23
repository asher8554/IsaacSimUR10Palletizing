# UR10 끝에 달린 흡착(Surface) 그리퍼를 열고 닫고 상태를 읽는다
"""로봇 USD(Gripper=short_suction) 안에 이미 들어 있는 것.

    ee_link/SurfaceGripper            그리퍼 본체 prim (IsaacSurfaceGripper)
    ee_link/suction_cup/Suction_Joint 붙는 지점(D6 조인트). 닫으면 이 조인트의 상대가 박스로 바뀐다.

닫기(close)를 하면 붙는 지점 앞 2 cm(maxGripDistance) 안에 있는 강체를 찾아 조인트로 묶는다.
열기(open)를 하면 조인트를 풀고, 박스는 중력으로 떨어진다. 즉 "붙였다 떼기"는 조인트를 만들었다 없애는 일이다.
"""

import numpy as np
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.robot.surface_gripper import _surface_gripper

from palletizing.common import make_T, quat_to_matrix, rigid_pose

# ee_link 원점에서 흡착컵 끝까지의 거리. ee_link의 +X 방향으로 17 cm.
TIP_OFFSET = np.array([0.17, 0.0, 0.0])


class SuctionGripper:
    def __init__(self, robot_path: str):
        self.path = f"{robot_path}/ee_link/SurfaceGripper"
        self._ee_link = RigidPrim(f"{robot_path}/ee_link")
        self._interface = None

    def initialize(self) -> None:
        """재생(Play) 중에 한 번 부른다."""
        self._interface = _surface_gripper.acquire_surface_gripper_interface()
        self.open()

    def close(self) -> None:
        """흡착 켜기. 닿은 물체가 있으면 붙는다."""
        self._interface.close_gripper(self.path)

    def open(self) -> None:
        """흡착 끄기. 붙어 있던 물체를 놓는다."""
        self._interface.open_gripper(self.path)

    def is_closed(self) -> bool:
        """물체를 붙잡은 상태인가. 닫으라고 했어도 닿은 물체가 없으면 False(Closing)다."""
        return self._interface.get_gripper_status(self.path) == _surface_gripper.GripperStatus.Closed

    def tip_T(self) -> np.ndarray:
        """흡착컵 끝의 월드 자세(4x4). x축이 컵이 향하는 방향이다."""
        p, q = rigid_pose(self._ee_link)
        R = quat_to_matrix(q)
        return make_T(p + R @ TIP_OFFSET, R)
