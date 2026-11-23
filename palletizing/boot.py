# Isaac Sim 앱을 띄우고 필요한 확장을 켜는 공통 시작 코드
"""모든 isaacsim.* / pxr import는 SimulationApp을 만든 뒤에만 가능하다. 그래서 이 파일을 맨 먼저 import한다."""

import argparse
import random

from isaacsim import SimulationApp

# 이 실습이 쓰는 확장. 컨베이어, 흡착 그리퍼, 모션 생성(RMPflow), 상태기계.
_EXTENSIONS = [
    "isaacsim.asset.gen.conveyor",
    "isaacsim.robot.surface_gripper",
    "isaacsim.robot_motion.cumotion",
    "isaacsim.robot_motion.experimental.motion_generation",
    "isaacsim.robot_motion.schema",
    "omni.isaac.core_archive",  # transitions(상태기계) 패키지가 들어 있다
]


def start_app(extra_config: dict | None = None):
    """인자를 읽고 SimulationApp을 띄운다. (app, args)를 돌려준다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="창 없이 실행")
    parser.add_argument("--test", action="store_true", help="자동 확인 후 종료")
    parser.add_argument("--seed", type=int, default=None, help="무작위 박스 위치를 고정")
    args, _ = parser.parse_known_args()

    config = {"headless": args.headless}
    if args.test:
        # 화면을 그리지 않아 시험이 빨리 끝난다.
        config.update({"renderer": "MinimalRendering", "disable_viewport_updates": True})
    config.update(extra_config or {})
    app = SimulationApp(config)

    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    for name in _EXTENSIONS:
        manager.set_extension_enabled_immediate(name, True)

    from isaacsim.core.simulation_manager import SimulationManager

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")  # 물리 60 Hz, CPU

    if args.seed is not None:
        random.seed(args.seed)
    return app, args


def wait_for_assets(app):
    """인터넷에서 받는 USD가 다 열릴 때까지 화면만 갱신한다."""
    import isaacsim.core.experimental.utils.stage as stage_utils

    app.update()
    while stage_utils.is_stage_loading():
        app.update()


def finish(app, args, passed: bool, name: str):
    """--test면 결과를 찍고 닫는다. 아니면 창을 닫을 때까지 돌린다."""
    if args.test:
        print(f"[{name}] {'PASS' if passed else 'FAIL'}", flush=True)
        app.close(exit_code=0 if passed else 1)
        return
    while app.is_running():
        app.update()
    app.close()
