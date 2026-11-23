# 7단계: 흡착컵 끝을 원하는 자세로 보낸다 (RMPflow, 접근 깔때기, 관절 홈 복귀)
"""실행: C:\\isaacsim-6.2\\python.bat step07_motion.py"""

from palletizing.boot import finish, start_app, wait_for_assets

app, args = start_app()

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
from isaacsim.core.simulation_manager import SimulationManager

from palletizing.common import matrix_to_quat, transforms_close
from palletizing.motion import HOME_JOINTS, NAVIGATION_OBSTACLES, MotionCommand, Ur10, create_planning_obstacles, make_approach
from palletizing.scene import ROBOT_PATH, build_scene

build_scene()
create_planning_obstacles()
wait_for_assets(app)

robot = Ur10(ROBOT_PATH)
robot.setup()  # 재생 전에
app_utils.play()
for _ in range(10):
    app.update()
robot.start()  # 재생 후에


def move(name: str, make_command, p_thresh: float, timeout: float = 8.0) -> bool:
    """명령을 매 스텝 새로 만들어 보내고, 도착하면 True를 돌려준다."""
    for frame in range(int(timeout * 60)):
        app.update()
        command = make_command()
        robot.send(command)
        robot.step(SimulationManager.get_physics_dt())
        if transforms_close(command.target_T(), robot.tip_T(), p_thresh, 2.0):
            error = np.linalg.norm(command.p - robot.tip_T()[:3, 3])
            print(f"  {name}: {frame / 60:.2f}초에 도착, 위치 오차 {error * 1000:.1f} mm")
            return True
    print(f"  {name}: {timeout}초 안에 도착하지 못함")
    return False


for _ in range(30):  # 홈 자세가 기록되도록 몇 스텝 돌린다
    app.update()
    robot.step(SimulationManager.get_physics_dt())
print("홈에서 컵 끝 위치:", np.round(robot.home_tip_T[:3, 3], 3))

DOWN = np.array([0.0, 0.0, -1.0])
# 자세: 컵(x축)이 아래를 보게. 열 순서는 x, y, z축.
q_down = matrix_to_quat(np.column_stack([DOWN, [0.0, 1.0, 0.0], np.cross(DOWN, [0.0, 1.0, 0.0])]))
pick_p = np.array([0.0, 0.70, -0.45])  # 컨베이어 끝, 박스 높이 근처
place_p = np.array([1.05, -0.31, -0.30])  # 팔레트 위

results = []

# (1) 컨베이어 위로. 위에서 곧게 내려가도록 깔때기를 쓴다.
results.append(move("컨베이어 위 집는 높이", lambda: MotionCommand(
    pick_p, q_down, make_approach(DOWN, np.linalg.norm(pick_p - robot.tip_T()[:3, 3]), 0.3, 0.005)), 0.01))  # fmt: skip

# (2) 수직으로 30 cm 들어 올린다.
lift_p = pick_p + np.array([0.0, 0.0, 0.3])
results.append(move("30 cm 들어 올리기", lambda: MotionCommand(lift_p, q_down), 0.02))

# (3) 팔레트 위로. 컨베이어에서 팔레트로 건너갈 때는 가상 장애물을 켜서 팔이 높이 돌아가게 한다.
robot.set_obstacles_enabled(NAVIGATION_OBSTACLES, True)
q_place = matrix_to_quat(np.column_stack([DOWN, [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]))
results.append(move("팔레트 위", lambda: MotionCommand(
    place_p, q_place, make_approach(DOWN, np.linalg.norm(place_p - robot.tip_T()[:3, 3]), 0.35, 0.005)), 0.02))  # fmt: skip
robot.set_obstacles_enabled(NAVIGATION_OBSTACLES, False)

# (4) 홈으로. 관절값으로 직접 돌아가야 팔꿈치가 항상 같은 모양으로 접힌다.
home_T = robot.home_tip_T
results.append(move("홈 복귀(관절 제어)", lambda: MotionCommand(
    home_T[:3, 3], matrix_to_quat(home_T[:3, :3]), joint_target=HOME_JOINTS), 0.005))  # fmt: skip

robot.send(None)
finish(app, args, all(results), "step07")
