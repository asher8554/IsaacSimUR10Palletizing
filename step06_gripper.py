# 6단계: 흡착 그리퍼로 박스를 붙였다가 떨어뜨린다
"""실행: C:\\isaacsim-6.2\\python.bat step06_gripper.py

로봇 모션은 다음 단계에서 만든다. 여기서는 박스를 흡착컵 바로 앞에 가져다 놓고
붙이기 → 로봇을 돌려도 따라오는지 → 떼기만 확인한다.
"""

from palletizing.boot import finish, start_app, wait_for_assets

app, args = start_app()

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.prims import Articulation, RigidPrim

from palletizing.common import matrix_to_quat, rigid_pose
from palletizing.gripper import SuctionGripper
from palletizing.scene import ENV_PATH, ROBOT_PATH, build_scene


def run_seconds(seconds: float) -> None:
    for _ in range(int(seconds * 60)):
        app.update()


urls = build_scene()
BIN_PATH = f"{ENV_PATH}/bins/test_bin"
stage_utils.add_reference_to_stage(usd_path=urls["bin"], path=BIN_PATH)
test_bin = RigidPrim(BIN_PATH, positions=[(3.0, 0.0, -1.0)], reset_xform_op_properties=True)  # 일단 멀리 둔다
wait_for_assets(app)

HOME = np.array([-1.57, -1.57, -1.57, -1.57, 1.57, 0.0])
robot = Articulation(ROBOT_PATH)
gripper = SuctionGripper(ROBOT_PATH)

app_utils.play()
run_seconds(0.2)
robot.set_dof_positions(HOME.tolist())
robot.set_dof_position_targets(HOME.tolist())
gripper.initialize()
run_seconds(1.0)

# 박스를 흡착컵 바로 앞에 놓는다. 박스 z축(바닥 → 입구)을 컵이 향하는 방향에 맞추면 바닥판이 컵을 마주 본다.
tip_T = gripper.tip_T()
cup_dir = tip_T[:3, 0]
print("흡착컵 끝 위치:", np.round(tip_T[:3, 3], 3), " 컵이 향하는 방향:", np.round(cup_dir, 2))
bin_R = np.column_stack([tip_T[:3, 1], np.cross(cup_dir, tip_T[:3, 1]), cup_dir])  # 열 = 박스의 x, y, z축
bin_center = tip_T[:3, 3] + cup_dir * (0.072 + 0.005)  # 바닥판이 중심에서 7.2 cm, 여유 5 mm
test_bin.set_world_poses(positions=[bin_center], orientations=[matrix_to_quat(bin_R)])
test_bin.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]])

# 붙이기.
gripper.close()
run_seconds(1.0)
attached = gripper.is_closed()
z_held = rigid_pose(test_bin)[0][2]
print(f"닫기 후 1초: 상태 {'Closed(붙음)' if attached else '붙지 않음'}, 박스 높이 z={z_held:.3f}")

# 로봇 첫 관절을 30도 돌린다. 붙어 있으면 박스가 따라온다.
target = HOME.copy()
target[0] += np.radians(30)
robot.set_dof_position_targets(target.tolist())
run_seconds(2.0)
gap = np.linalg.norm(rigid_pose(test_bin)[0] - gripper.tip_T()[:3, 3])
print(f"로봇 회전 후: 컵 끝과 박스 중심 거리 {gap:.3f} m (붙어 있으면 약 0.08)")

# 떼기.
gripper.open()
run_seconds(0.5)
print(f"열기 후 0.5초(깨우기 전): 박스 높이 z={rigid_pose(test_bin)[0][2]:.3f}")
# 가만히 매달려 있던 강체는 물리 엔진이 "잠재워" 둔다. 조인트가 풀려도 스스로 깨어나지 않으므로 살짝 건드려 깨운다.
test_bin.set_velocities(linear_velocities=[[0.0, 0.0, -0.01]])
run_seconds(1.0)
z_dropped = rigid_pose(test_bin)[0][2]
print(f"열기 후 1.5초: 상태 {'Closed' if gripper.is_closed() else 'Open(놓음)'}, 박스 높이 z={z_dropped:.3f}")

passed = attached and gap < 0.12 and not gripper.is_closed() and z_dropped < z_held - 0.3
finish(app, args, passed, "step06")
