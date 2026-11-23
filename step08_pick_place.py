# 8단계: 박스 하나를 컨베이어에서 집어 팔레트에 놓는다 (동작 조각을 순서대로 실행)
"""실행: C:\\isaacsim-6.2\\python.bat step08_pick_place.py

뒤집기는 다음 단계에서 다룬다. 여기서는 박스를 항상 엎어서(바로 쌓을 수 있게) 내보낸다.
"""

from dataclasses import replace

from palletizing.boot import finish, start_app, wait_for_assets

app, args = start_app()

import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.simulation_manager import SimulationManager

from palletizing.cell import PalletizingCell
from palletizing.common import rigid_pose
from palletizing.motion import Ur10, create_planning_obstacles
from palletizing.scene import ROBOT_PATH, build_scene, configure_conveyor
from palletizing.spawner import BinSpawner
from palletizing.tasks import Command, pick_sequence, place_sequence

urls = build_scene()
create_planning_obstacles()
wait_for_assets(app)
configure_conveyor()

robot = Ur10(ROBOT_PATH)
robot.setup()
spawner = BinSpawner(urls["bin"], upside_down_probability=1.0)  # 전부 엎어서
cell = PalletizingCell(robot, spawner, max_bins=1)

app_utils.play()
for _ in range(10):
    app.update()
robot.start()
cell.reset()

sequence = []  # 실행할 동작 조각 목록
index = 0
done = False
frame = 0
while app.is_running() and not done:
    app.update()
    if not app_utils.is_playing():
        continue
    frame += 1
    if args.test and frame > 2400:
        break

    if not cell.stacked:  # 박스 하나만 다룬다
        spawner.step()
    obs = cell.read(SimulationManager.get_simulation_time())

    # 박스가 작업 구역에 들어오면 "집기 + 놓기" 순서를 한 번 만든다.
    if not sequence and obs.has_active_bin:
        sequence = pick_sequence(obs) + place_sequence()
        sequence[0].enter(obs)
        print(f"<작업 시작: 동작 {len(sequence)}개>")

    command = Command()
    if sequence and index < len(sequence):
        finished, command = sequence[index].step(obs)
        if finished:
            print(f"  {index + 1}/{len(sequence)} {type(sequence[index]).__name__} 완료 (t={obs.time:.2f}s)")
            index += 1
            if index < len(sequence):
                sequence[index].enter(obs)
            # 흡착을 끄는 순간 박스가 제자리에 있었다면 "쌓음"으로 기록한다.
            if type(sequence[index - 1]).__name__ == "OpenGripper" and obs.bin_at_stack_target:
                command = replace(command, complete_bin=True)
            done = index == len(sequence)
    cell.apply(command, SimulationManager.get_physics_dt())

passed = False
if cell.stacked:
    p = rigid_pose(cell.stacked[0].rigid)[0]
    error = p[:2] - cell.targets[0][:2]
    print(f"놓인 위치 ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}), 목표와의 수평 오차 {abs(error).max() * 1000:.0f} mm")
    passed = done and abs(error).max() < 0.03
finish(app, args, passed, "step08")
