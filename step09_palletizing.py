# 9단계(완성): 무작위 박스를 집고, 필요하면 뒤집고, 팔레트에 차례로 쌓는다
"""실행: C:\\isaacsim-6.2\\python.bat step09_palletizing.py
옵션: --seed 1 (박스 순서 고정), --max-bins 3 (3개만 쌓고 끝)
"""

import argparse

from palletizing.boot import finish, start_app, wait_for_assets

parser = argparse.ArgumentParser()
parser.add_argument("--max-bins", type=int, default=None, help="쌓을 박스 수(기본 9)")
options, _ = parser.parse_known_args()

app, args = start_app()

import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.simulation_manager import SimulationManager

from palletizing.cell import PalletizingCell
from palletizing.fsm import PalletizerController
from palletizing.motion import Ur10, create_planning_obstacles
from palletizing.scene import ROBOT_PATH, build_scene, configure_conveyor
from palletizing.spawner import BinSpawner

# 장면
urls = build_scene()
create_planning_obstacles()
wait_for_assets(app)
configure_conveyor()

# 구성 요소
robot = Ur10(ROBOT_PATH)
robot.setup()
spawner = BinSpawner(urls["bin"])
cell = PalletizingCell(robot, spawner, max_bins=options.max_bins)
controller = PalletizerController()

app_utils.play()
for _ in range(10):
    app.update()
robot.start()
cell.reset()

visited = set()
frame = 0
frame_limit = 2400 * len(cell.targets)  # 박스 하나에 40초까지
while app.is_running() and controller.state != "done":
    app.update()
    if not app_utils.is_playing():
        continue
    frame += 1
    if args.test and frame > frame_limit:
        break

    spawner.step()  # 1. 구역이 비면 박스 생성
    obs = cell.read(SimulationManager.get_simulation_time())  # 2. 관측
    command = controller.step(obs)  # 3. 판단
    cell.apply(command, SimulationManager.get_physics_dt())  # 4. 실행
    visited.add(controller.state)

print(f"<쌓은 박스 {len(cell.stacked)}/{len(cell.targets)}, 거친 상태 {sorted(visited)}, {frame / 60:.0f}초>")
finish(app, args, controller.state == "done" and len(cell.stacked) == len(cell.targets), "step09")
