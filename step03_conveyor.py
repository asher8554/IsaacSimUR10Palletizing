# 3단계: 컨베이어를 구동하고 박스 하나를 올려 실려 가는지 본다
"""실행: C:\\isaacsim-6.2\\python.bat step03_conveyor.py"""

from palletizing.boot import finish, start_app, wait_for_assets

app, args = start_app()

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import RigidPrim

from palletizing.scene import ENV_PATH, build_scene, configure_conveyor, set_conveyor_running

urls = build_scene()
wait_for_assets(app)
configure_conveyor()  # 에셋이 다 열린 뒤에 불러야 한다

# 박스 하나를 컨베이어 입구 위에 놓는다. 박스 USD는 이미 강체(RigidBody)로 만들어져 있다.
BIN_PATH = f"{ENV_PATH}/bins/test_bin"
stage_utils.add_reference_to_stage(usd_path=urls["bin"], path=BIN_PATH)
test_bin = RigidPrim(BIN_PATH, positions=[(0.0, 1.5, -0.15)], reset_xform_op_properties=True)
wait_for_assets(app)

app_utils.play()


def run_seconds(seconds: float) -> float:
    """seconds 동안 시뮬레이션을 돌리고 박스의 y 위치를 돌려준다."""
    for _ in range(int(seconds * 60)):
        app.update()
    return float(test_bin.get_world_poses()[0].numpy()[0][1])


y0 = run_seconds(0.5)  # 떨어져서 벨트에 앉을 시간
y1 = run_seconds(1.0)
print(f"구동 1초: y {y0:.3f} → {y1:.3f}  (이동 {y0 - y1:.3f} m, 기대값 약 0.30)")

set_conveyor_running(False)
y2 = run_seconds(0.5)  # 멈추는 중
y3 = run_seconds(1.0)
print(f"정지 1초: y {y2:.3f} → {y3:.3f}  (이동 {y2 - y3:.3f} m, 기대값 약 0)")

set_conveyor_running(True)
passed = 0.2 < (y0 - y1) < 0.4 and abs(y2 - y3) < 0.02
finish(app, args, passed, "step03")
