# 가이드에 넣을 화면을 찍는다 (전체 작업을 돌리며 주요 순간마다 뷰포트를 저장)
"""실행(저장소 루트에서): C:\\isaacsim-6.2\\python.bat docs\\capture_shots.py --headless --seed 5"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from palletizing.boot import start_app, wait_for_assets

app, args = start_app({"width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from PIL import Image

from palletizing.cell import PalletizingCell
from palletizing.fsm import PalletizerController
from palletizing.motion import Ur10, create_planning_obstacles
from palletizing.scene import ROBOT_PATH, build_scene, configure_conveyor
from palletizing.spawner import BinSpawner

OUT = ROOT / "docs" / "img"
OUT.mkdir(exist_ok=True)
OVERVIEW = dict(eye=[3.0, 3.0, 1.5], target=[0.35, 0.2, -0.4])
CLOSE = dict(eye=[1.9, 2.1, 0.5], target=[0.3, 0.5, -0.45])


def shot(name: str, view: dict = OVERVIEW) -> None:
    """카메라를 맞추고 렌더가 가라앉기를 기다렸다가 저장한다. 시뮬레이션은 멈춰 둔다."""
    was_playing = app_utils.is_playing()
    if was_playing:
        app_utils.pause()
    ViewportManager.set_camera_view("/OmniverseKit_Persp", **view)
    for _ in range(40):
        app.update()
    png = OUT / f"{name}.png"
    capture_viewport_to_file(get_active_viewport(), str(png))
    for _ in range(10):
        app.update()
    Image.open(png).convert("RGB").save(OUT / f"{name}.jpg", quality=85)
    png.unlink()
    print(f"<저장 {name}.jpg>", flush=True)
    if was_playing:
        app_utils.play()
        app.update()


urls = build_scene()
create_planning_obstacles()
wait_for_assets(app)
configure_conveyor()
robot = Ur10(ROBOT_PATH)
robot.setup()
spawner = BinSpawner(urls["bin"])
cell = PalletizingCell(robot, spawner, max_bins=4)
controller = PalletizerController()

app_utils.play()
for _ in range(10):
    app.update()
robot.start()
cell.reset()
for _ in range(60):
    app.update()
    robot.step(SimulationManager.get_physics_dt())
shot("scene")

taken = set()
previous_state = "idle"
attached_frames = 0
frame = 0
while app.is_running() and controller.state != "done" and frame < 2400 * 4:
    app.update()
    if not app_utils.is_playing():
        continue
    frame += 1
    spawner.step()
    obs = cell.read(SimulationManager.get_simulation_time())
    command = controller.step(obs)
    cell.apply(command, SimulationManager.get_physics_dt())

    state = controller.state
    attached_frames = attached_frames + 1 if obs.bin_attached else 0
    if "bin_arrived" not in taken and state == "picking":
        taken.add("bin_arrived")
        shot("bin_arrived", CLOSE)
    if "attached" not in taken and attached_frames == 25:
        taken.add("attached")
        shot("attached", CLOSE)
    if "flipping" not in taken and previous_state == "flipping" and state == "picking":
        taken.add("flipping")
        shot("flipped")
    if "placing" not in taken and state == "placing" and obs.bin_at_stack_target:
        taken.add("placing")
        shot("placing")
    previous_state = state

for _ in range(60):
    app.update()
shot("stacked")
print(f"<끝: 상태 {controller.state}, 쌓은 박스 {len(cell.stacked)}>", flush=True)
app.close()
