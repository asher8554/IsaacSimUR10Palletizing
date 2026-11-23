# 위치·자세 계산에 쓰는 작은 수학 도구 모음
"""자세 표현 두 가지를 오간다.

    쿼터니언 q = (w, x, y, z)   Isaac Sim이 주고받는 형식
    4x4 변환행렬 T               왼쪽 위 3x3이 회전 R, 오른쪽 열이 위치 p
                                 R의 1·2·3번째 열이 그 물체의 x·y·z축이 월드에서 향하는 방향이다.
"""

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from pxr import Usd, UsdGeom
from scipy.spatial.transform import Rotation


def quat_to_matrix(q) -> np.ndarray:
    """(w,x,y,z) → 3x3 회전행렬. scipy는 (x,y,z,w) 순서라 자리를 바꿔 넣는다."""
    return Rotation.from_quat(np.asarray(q)[[1, 2, 3, 0]]).as_matrix()


def matrix_to_quat(R) -> np.ndarray:
    """3x3 회전행렬 → (w,x,y,z)."""
    return Rotation.from_matrix(R).as_quat()[[3, 0, 1, 2]]


def make_T(p, R) -> np.ndarray:
    """위치 p와 회전 R로 4x4 변환행렬을 만든다."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def read_world_pose(prim_path: str):
    """USD에서 prim의 월드 위치와 쿼터니언을 읽는다. 강체가 아닌 하위 prim도 읽을 수 있다."""
    stage = stage_utils.get_current_stage(backend="usd")
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    xform.Orthonormalize()  # 스케일을 떼어 내고 순수 회전만 남긴다
    q = xform.ExtractRotationQuat()
    return np.array(xform.ExtractTranslation(), dtype=float), np.array([q.GetReal(), *q.GetImaginary()], dtype=float)


def rigid_pose(rigid_prim):
    """RigidPrim의 월드 위치와 쿼터니언을 물리 엔진에서 읽는다."""
    positions, orientations = rigid_prim.get_world_poses()
    return np.asarray(positions.numpy()[0], dtype=float), np.asarray(orientations.numpy()[0], dtype=float)


def transforms_close(T1, T2, p_thresh: float, R_thresh: float) -> bool:
    """두 자세가 위치 p_thresh(m), 회전 R_thresh 이내로 가까운가."""
    diff = T1 - T2
    return bool(np.linalg.norm(diff[:3, 3]) <= p_thresh and np.linalg.norm(diff[:3, :3]) / 3 <= R_thresh)
