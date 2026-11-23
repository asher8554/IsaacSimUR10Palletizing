# 4단계: 박스를 무작위 위치·자세로 계속 만들어 컨베이어에 올린다
"""실행: C:\\isaacsim-6.2\\python.bat step04_spawn_bins.py --seed 3

아직 로봇이 박스를 집지 않으므로, 박스가 작업 구역(y < 0.7)에 들어오면 옆으로 치워서
다음 박스가 나오게 한다. 이 "치우기"가 뒤 단계에서 로봇이 할 일이다.
"""

from palletizing.boot import finish, start_app, wait_for_assets

app, args = start_app()

import isaacsim.core.experimental.utils.app as app_utils

from palletizing.common import rigid_pose
from palletizing.scene import build_scene, configure_conveyor
from palletizing.spawner import BinSpawner

urls = build_scene()
wait_for_assets(app)
configure_conveyor()

spawner = BinSpawner(urls["bin"])
app_utils.play()

removed = 0
MAX_BINS = 3 if args.test else 1000
while app.is_running() and removed < MAX_BINS:
    app.update()
    if not app_utils.is_playing():
        continue
    spawner.step()  # 구역이 비었으면 새 박스를 만든다
    position, _ = rigid_pose(spawner.on_conveyor)
    if position[1] < 0.7:  # 작업 구역 도착
        print(f"  도착 위치 x={position[0]:+.3f}, y={position[1]:.3f}, z={position[2]:.3f} → 치움")
        spawner.on_conveyor.set_world_poses(positions=[(3.0 + removed, 1.0, -0.9)])
        spawner.on_conveyor.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]])
        removed += 1

finish(app, args, len(spawner.bins) == removed == MAX_BINS, "step04")
