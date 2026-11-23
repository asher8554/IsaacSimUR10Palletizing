# 5단계: 작업 구역에 들어온 박스의 위치·자세를 읽어 집을 자세와 뒤집기 여부를 계산한다
"""실행: C:\\isaacsim-6.2\\python.bat step05_perception.py --seed 3"""

from palletizing.boot import finish, start_app, wait_for_assets

app, args = start_app()

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np

from palletizing.perception import BinState, estimate_grasp, find_active_bin
from palletizing.scene import build_scene, configure_conveyor
from palletizing.spawner import BinSpawner

urls = build_scene()
wait_for_assets(app)
configure_conveyor()

spawner = BinSpawner(urls["bin"])
bins: list[BinState] = []
app_utils.play()

checked, correct = 0, 0
MAX_BINS = 4 if args.test else 1000
while app.is_running() and checked < MAX_BINS:
    app.update()
    if not app_utils.is_playing():
        continue
    spawner.step()
    for rigid in spawner.bins[len(bins) :]:  # 새로 생긴 박스를 추적 목록에 올린다
        bins.append(BinState(rigid))

    active = find_active_bin(bins)
    if active is None:
        continue

    estimate_grasp(active)
    p = active.grasp_T[:3, 3]
    approach = active.grasp_T[:3, 0]
    yaw = np.degrees(np.arctan2(active.grasp_T[1, 1], active.grasp_T[0, 1]))  # 집는 자세 y축의 방위각
    truth_upside_down = spawner.upside_down_log[bins.index(active)]
    ok = active.needs_flip == (not truth_upside_down)
    print(
        f"  인식: 집을 점 ({p[0]:+.3f}, {p[1]:.3f}, {p[2]:.3f}), 접근 방향 {np.round(approach, 2)}, "
        f"회전 {yaw:+.0f}도, 뒤집기 {'필요' if active.needs_flip else '불필요'} → {'정답' if ok else '오답'}"
    )
    checked += 1
    correct += ok
    active.rigid.set_world_poses(positions=[(3.0 + checked, 1.0, -0.9)])  # 치우기(로봇이 할 일을 대신)
    active.rigid.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]])

finish(app, args, checked == correct == MAX_BINS, "step05")
