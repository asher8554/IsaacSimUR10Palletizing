# 무작위로 놓인 박스의 위치·자세를 읽어 "어디를 어느 방향으로 집을지"를 계산한다
"""여기서 "인식"은 카메라 영상 처리가 아니다. 시뮬레이터가 알고 있는 박스의 월드 포즈를 그대로 읽는다.
실제 설비라면 이 자리에 3D 비전(포즈 추정)이 들어가고, 그 뒤의 계산은 똑같이 쓸 수 있다.
"""

from dataclasses import dataclass

import numpy as np
from isaacsim.core.experimental.prims import RigidPrim

from palletizing.common import make_T, quat_to_matrix, read_world_pose, rigid_pose

# 작업 구역: 컨베이어 끝부분. 이 안에 들어온 박스만 집는다.
WORK_ZONE_Y = (0.0, 0.7)
WORK_ZONE_X = (-0.4, 0.4)
_GRASP_Y_REFERENCE = np.array([-0.15, 1.0, 0.0])  # 집는 자세의 y축이 향했으면 하는 쪽


@dataclass
class BinState:
    """박스 하나에 대해 알아낸 것."""

    rigid: RigidPrim
    grasp_T: np.ndarray | None = None  # 집을 자세(4x4). x축이 흡착컵이 다가갈 방향
    needs_flip: bool | None = None  # 입구가 위를 향해 있어서 뒤집어야 하는가
    is_attached: bool = False  # 지금 흡착컵에 붙어 있는가

    @property
    def base_path(self) -> str:
        """박스 바닥판 prim. 흡착컵이 붙을 평평한 면이다."""
        return self.rigid.paths[0] + "/Collision/Cube_03"


def find_active_bin(bins: list[BinState]) -> BinState | None:
    """작업 구역 안의 박스 중 로봇에 가장 가까운(y가 가장 작은) 것을 고른다."""
    candidates = []
    for bin_state in bins:
        x, y, _ = rigid_pose(bin_state.rigid)[0]
        if WORK_ZONE_Y[0] < y < WORK_ZONE_Y[1] and WORK_ZONE_X[0] < x < WORK_ZONE_X[1]:
            candidates.append((y, bin_state))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def estimate_grasp(bin_state: BinState, up_vec=(0.0, 0.0, 1.0)) -> None:
    """바닥판 포즈로부터 집을 자세(grasp_T)와 뒤집기 필요 여부를 계산해 bin_state에 적는다.

    up_vec: "위쪽" 기준. 컨베이어 위에서는 월드 +Z, 로봇이 들고 있을 때는 흡착컵 반대 방향을 넣는다.
    """
    base_p, base_q = read_world_pose(bin_state.base_path)
    R = quat_to_matrix(base_q)
    bin_x, bin_z = R[:, 0], R[:, 2]  # 박스의 x축, z축(바닥 → 입구 방향)이 월드에서 향하는 곳

    # 박스 z축이 위를 보면 입구가 위. 흡착할 평평한 바깥 바닥이 아래 깔려 있으니 뒤집어야 쌓을 수 있다.
    bin_state.needs_flip = float(np.dot(up_vec, bin_z)) > 0.0

    if bin_state.needs_flip:
        approach = -bin_z  # 입구로 들어가 안쪽 바닥을 집는다
        margin = 0.0025  # 바닥판 두께만큼 안쪽 면으로
    else:
        approach = bin_z  # 위로 드러난 바깥 바닥을 집는다
        margin = -0.0025

    # 집는 자세의 y축은 박스 x축에 맞춘다. 박스는 180도 돌려도 같으므로 +bin_x, -bin_x 둘 다 답이다.
    # 그중 월드 +Y 쪽을 골라 손목이 덜 꼬이게 한다. 기준을 -X 쪽으로 조금 기울여 둔 것은,
    # 컨베이어 끝에 반듯하게 걸린 박스(bin_x가 월드 X와 나란함)에서 부호가 잡음으로 뒤집히지 않게 하려는 것이다.
    grasp_y = bin_x if np.dot(bin_x, _GRASP_Y_REFERENCE) > 0.0 else -bin_x
    grasp_z = np.cross(approach, grasp_y)
    bin_state.grasp_T = make_T(base_p + margin * bin_z, np.column_stack([approach, grasp_y, grasp_z]))
