# 컨베이어 입구에 박스를 무작위 위치·자세로 하나씩 만들어 내는 생성기
import random

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.prims import RigidPrim
from scipy.spatial.transform import Rotation

from palletizing.common import rigid_pose
from palletizing.scene import CONVEYOR_SPEED, ENV_PATH


def random_bin_pose(upside_down_probability: float = 0.5):
    """박스가 떨어질 무작위 위치와 자세를 만든다. (위치, 쿼터니언, 엎어졌는지)를 돌려준다."""
    # 컨베이어 폭 방향(x)으로 ±15 cm 흩뜨리고, 입구(y=1.5) 위 살짝 뜬 높이에서 떨어뜨린다.
    position = np.array([random.uniform(-0.15, 0.15), 1.5, -0.15])
    # 수직축 둘레로 아무 각도나.
    rotation = Rotation.from_euler("z", random.uniform(-np.pi, np.pi))
    # 절반은 엎어서(입구가 아래로) 내보낸다.
    upside_down = random.random() < upside_down_probability
    if upside_down:
        rotation = rotation * Rotation.from_euler("y", np.pi)
    return position, rotation.as_quat()[[3, 0, 1, 2]], upside_down


class BinSpawner:
    """작업 구역이 비면 새 박스를 하나 만든다."""

    def __init__(self, bin_usd: str, env_path: str = ENV_PATH, upside_down_probability: float = 0.5):
        self.bin_usd = bin_usd
        self.env_path = env_path
        self.upside_down_probability = upside_down_probability
        self.bins: list[RigidPrim] = []  # 지금까지 만든 박스 전부
        self.on_conveyor: RigidPrim | None = None  # 컨베이어 위에 있는 가장 최근 박스
        self.upside_down_log: list[bool] = []  # 만들 때 엎었는지 기록(인식 결과를 검산할 정답지)

    def step(self) -> None:
        """매 물리 스텝마다 부른다. 컨베이어 위 박스가 아직 있으면 아무것도 안 한다."""
        if self.on_conveyor is not None:
            x, y, _ = rigid_pose(self.on_conveyor)[0]
            if y > 0.0 and -0.4 < x < 0.4:  # 아직 컨베이어 구역 안
                return

        prim_path = f"{self.env_path}/bins/bin_{len(self.bins)}"
        position, orientation, upside_down = random_bin_pose(self.upside_down_probability)
        print(f"<박스 생성 {prim_path.split('/')[-1]}: x={position[0]:+.3f}, {'엎어짐' if upside_down else '바로 놓임'}>")

        stage_utils.add_reference_to_stage(usd_path=self.bin_usd, path=prim_path)
        rigid_bin = RigidPrim(prim_path, positions=[position], orientations=[orientation], reset_xform_op_properties=True)
        # 물리 엔진 쪽 위치도 같은 값으로 맞추고, 컨베이어와 같은 속도를 미리 줘서 미끄러짐을 줄인다.
        rigid_bin.set_world_poses(positions=[position], orientations=[orientation])
        rigid_bin.set_velocities(linear_velocities=[[0.0, -CONVEYOR_SPEED, 0.0]])
        self.on_conveyor = rigid_bin
        self.bins.append(rigid_bin)
        self.upside_down_log.append(upside_down)

    def reset(self) -> None:
        """만든 박스를 모두 지운다."""
        stage = stage_utils.get_current_stage(backend="usd")
        for rigid_bin in self.bins:
            if stage.GetPrimAtPath(rigid_bin.paths[0]).IsValid():
                stage_utils.delete_prim(rigid_bin.paths[0])
        self.bins.clear()
        self.upside_down_log.clear()
        self.on_conveyor = None
