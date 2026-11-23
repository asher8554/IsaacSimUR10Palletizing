# 2단계: 로봇 주변에 컨베이어, 뒤집기 장치, 팔레트 등 설비를 배치한다
"""실행: C:\\isaacsim-6.2\\python.bat step02_build_scene.py"""

from palletizing.boot import finish, start_app, wait_for_assets

app, args = start_app()

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from pxr import Usd, UsdGeom

from palletizing.scene import ENV_PATH, build_scene

build_scene()
wait_for_assets(app)

# 배치한 설비가 실제로 어디에 얼마만 한 크기로 놓였는지 찍어 본다.
stage = stage_utils.get_current_stage(backend="usd")
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
passed = True
for name in ["ur10", "ur10_mount", "housing", "conveyor", "flip_station", "pallet", "dolly"]:
    prim = stage.GetPrimAtPath(f"{ENV_PATH}/{name}")
    box = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = box.GetMin(), box.GetMax()
    print(f"{name:13s} x[{lo[0]:+.2f},{hi[0]:+.2f}] y[{lo[1]:+.2f},{hi[1]:+.2f}] z[{lo[2]:+.2f},{hi[2]:+.2f}]")
    passed = passed and prim.IsValid() and not box.IsEmpty()

app_utils.play()
finish(app, args, passed, "step02")
