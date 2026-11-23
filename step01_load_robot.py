# 1단계: 빈 스테이지에 물리를 켜고 외부 UR10 로봇을 불러와 홈 자세로 세운다
"""실행: C:\\isaacsim-6.2\\python.bat step01_load_robot.py

이 파일만은 공통 코드(palletizing/boot.py) 없이 전부 풀어 썼다. Isaac Sim 스크립트의 뼈대를 한 번 보기 위함이다.
"""

import argparse

# (1) 앱부터 띄운다. 이 줄보다 위에서 isaacsim.* 나 pxr 을 import 하면 실패한다.
from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--test", action="store_true")
args, _ = parser.parse_known_args()
simulation_app = SimulationApp({"headless": args.headless})

# (2) 앱이 뜬 뒤에 나머지를 import 한다.
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

# (3) 물리 설정. 1/60초마다 한 스텝, CPU에서 계산.
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")

# (4) 바닥과 조명.
GroundPlane("/World/Ground")
DomeLight("/World/DomeLight").set_intensities(1000)

# (5) 외부 로봇 불러오기. USD 파일을 "reference"로 스테이지의 한 경로에 붙인다.
#     variants 로 흡착 그리퍼가 달린 형태를 고른다. (Gripper: none / short_suction / long_suction)
ROBOT_PATH = "/World/ur10"
ur10_usd = get_assets_root_path() + "/Isaac/Robots_Multiphysics/UniversalRobots/ur10/ur10.usda"
stage_utils.add_reference_to_stage(usd_path=ur10_usd, path=ROBOT_PATH, variants=[("Gripper", "short_suction")])
XformPrim(ROBOT_PATH, translations=[(0.0, 0.0, 0.0)], reset_xform_op_properties=True)

# (6) 인터넷에서 받는 중이면 다 열릴 때까지 기다린다.
simulation_app.update()
while stage_utils.is_stage_loading():
    simulation_app.update()

# (7) 관절 로봇으로 다루기 위한 래퍼. 홈 자세(라디안)를 기본 상태로 등록한다.
HOME = np.array([-1.57, -1.57, -1.57, -1.57, 1.57, 0.0])
robot = Articulation(ROBOT_PATH)
robot.set_default_state(dof_positions=[HOME])

# (8) 재생(Play)을 누른 것과 같다. 물리가 돌기 시작한다.
app_utils.play()
for _ in range(10):
    simulation_app.update()

print("관절 이름:", robot.dof_names)
print("링크 이름:", robot.link_names)

# (9) 홈 자세로 순간 이동시키고, 구동기 목표도 같은 값으로 맞춘다.
#     목표를 안 맞추면 구동기가 예전 목표(0)로 다시 끌고 간다.
robot.set_dof_positions(HOME.tolist())
robot.set_dof_position_targets(HOME.tolist())

for _ in range(120):  # 2초
    simulation_app.update()

q = robot.get_dof_positions().numpy()[0]
print("현재 관절각(rad):", np.round(q, 3))
passed = len(robot.dof_names) == 6 and np.allclose(q, HOME, atol=0.02)

if args.test:
    print(f"[step01] {'PASS' if passed else 'FAIL'}", flush=True)
else:
    while simulation_app.is_running():  # 창을 닫을 때까지
        simulation_app.update()
simulation_app.close()
