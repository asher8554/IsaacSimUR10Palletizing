# 팔레타이징 셀의 설비(로봇, 컨베이어, 뒤집기 장치, 팔레트)를 배치하고 컨베이어를 구동한다
"""좌표 기준: 로봇 베이스가 원점(0,0,0), 단위는 m, 회전은 쿼터니언 (w, x, y, z).

    +Y  컨베이어가 박스를 싣고 오는 쪽 (박스는 -Y 방향으로 흘러온다)
    +X  뒤집기 장치와 팔레트가 있는 쪽
    바닥은 z = -1.18 (로봇은 받침대 위에 서 있다)
"""

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.asset.gen.conveyor import create_conveyor_belt
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ENV_PATH = "/World/Cell"  # 셀 전체를 담는 상위 경로
ROBOT_PATH = f"{ENV_PATH}/ur10"
CONVEYOR_SPEED = 0.30  # m/s

_Z_ROT_M90 = (0.7071068, 0.0, 0.0, -0.7071068)  # Z축 -90도
_Z_ROT_P90 = (0.7071068, 0.0, 0.0, 0.7071068)  # Z축 +90도


def asset_urls() -> dict[str, str]:
    """에셋 서버에 있는 USD 주소 모음."""
    root = get_assets_root_path()
    if root is None:
        raise RuntimeError("에셋 서버를 찾지 못했다. 인터넷 연결을 확인한다.")
    return {
        "ur10": root + "/Isaac/Robots_Multiphysics/UniversalRobots/ur10/ur10.usda",
        "mount": root + "/Isaac/Props/Mounts/ur10_mount.usd",
        "housing": root + "/Isaac/Props/Sortbot_Housing/sortbot_housing.usd",
        "conveyor": root + "/Isaac/Props/Conveyors/ConveyorBelt_A05.usd",
        "flip_station": root + "/Isaac/Props/Flip_Stack/flip_stack.usd",
        "pallet": root + "/Isaac/Props/Pallet/pallet.usd",
        "dolly": root + "/Isaac/Props/Dolly/dolly.usd",
        "bin": root + "/Isaac/Props/KLT_Bin/small_KLT.usd",
        "warehouse": root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    }


def add_usd(usd_path, prim_path, translation, orientation=(1.0, 0.0, 0.0, 0.0), variants=None):
    """USD 파일을 prim_path에 reference로 붙이고 위치·자세를 정한다."""
    stage_utils.add_reference_to_stage(usd_path=usd_path, path=prim_path, variants=variants)
    XformPrim(prim_path, translations=[translation], orientations=[orientation], reset_xform_op_properties=True)


def build_scene(env_path: str = ENV_PATH) -> dict[str, str]:
    """셀을 만든다. 쓰인 에셋 주소 dict를 돌려준다(박스 생성기가 다시 쓴다)."""
    urls = asset_urls()
    stage = stage_utils.get_current_stage(backend="usd")
    stage_utils.define_prim(env_path)

    # 컨베이어. 불러오는 것과 kinematic 지정을 한 묶음으로 처리한다(아래 함수 설명 참고).
    stage.DefinePrim(f"{env_path}/conveyor", "Xform")
    with Sdf.ChangeBlock():
        add_usd(urls["conveyor"], f"{env_path}/conveyor", (-0.0544, 2.409, -1.4), _Z_ROT_M90)
        _keep_rollers_kinematic(stage, f"{env_path}/conveyor/Rollers")

    # 뒤집기 장치, 팔레트, 팔레트 대차.
    add_usd(urls["flip_station"], f"{env_path}/flip_station", (0.9087, 0.6629, -0.9453), _Z_ROT_M90)
    add_usd(urls["pallet"], f"{env_path}/pallet", (0.8826, -0.3281, -0.8098), _Z_ROT_P90)
    add_usd(urls["dolly"], f"{env_path}/dolly", (0.8875, -0.3329, -1.1811))

    # 로봇 받침대, 로봇을 둘러싼 하우징, 로봇.
    add_usd(urls["mount"], f"{env_path}/ur10_mount", (0.0, 0.0, 0.0))
    add_usd(urls["housing"], f"{env_path}/housing", (-0.018, -0.282, -1.175))
    add_usd(urls["ur10"], f"{env_path}/ur10", (0.0, 0.0, 0.0), variants=[("Gripper", "short_suction")])

    # 보이지 않는 물리 바닥. 떨어진 박스가 끝없이 추락하지 않게 한다.
    ground = GroundPlane(f"{env_path}/ground", sizes=50.0, translations=[(0.0, 0.0, -1.1819)], templates=None)
    ground.set_visibilities(False)

    # 로봇과 하우징은 서로 충돌 계산에서 뺀다. 로봇이 하우징 안쪽에 서 있어 팔이 하우징에 닿을 수 있다.
    robot_group = UsdPhysics.CollisionGroup.Define(stage, f"{env_path}/RobotGroup")
    robot_group.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(Sdf.Path(f"{env_path}/ur10"))
    housing_group = UsdPhysics.CollisionGroup.Define(stage, f"{env_path}/HousingGroup")
    housing_group.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(Sdf.Path(f"{env_path}/housing"))
    robot_group.CreateFilteredGroupsRel().AddTarget(housing_group.GetPrim().GetPath())

    # 배경(창고)과 조명. 물리와는 무관하다.
    stage_utils.add_reference_to_stage(usd_path=urls["warehouse"], path="/World/Background")
    XformPrim("/World/Background", positions=[10.0, 2.0, -1.1818], orientations=[_Z_ROT_P90], reset_xform_op_properties=True)
    light = DistantLight("/World/DistantLight", angles=np.deg2rad(1.0), orientations=[(0.9238795, 0.0, 0.3826834, 0.0)])
    light.set_intensities(3000)
    return urls


def _keep_rollers_kinematic(stage: Usd.Stage, rollers_path: str) -> None:
    """롤러를 kinematic(물리에 밀리지 않는 강체)으로 미리 적어 둔다.

    컨베이어는 "움직이지 않지만 표면 속도를 가진 강체"여야 한다. reference가 열리기 전에
    이 값을 적어 두지 않으면 롤러가 중력을 받는 일반 강체로 읽힐 수 있다.
    """
    layer = stage.GetEditTarget().GetLayer()
    spec = Sdf.CreatePrimInLayer(layer, stage.GetEditTarget().MapToSpecPath(Sdf.Path(rollers_path)))
    attr = Sdf.AttributeSpec(spec, "physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, Sdf.VariabilityUniform)
    attr.default = True


def configure_conveyor(env_path: str = ENV_PATH) -> None:
    """에셋이 다 열린 뒤 호출한다. 롤러에 컨베이어 동작(표면 속도)을 붙인다."""
    stage = stage_utils.get_current_stage(backend="usd")
    rollers = stage.GetPrimAtPath(f"{env_path}/conveyor/Rollers")
    if not rollers.IsValid() or not any(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(rollers)):
        raise RuntimeError("컨베이어 롤러가 아직 열리지 않았다. wait_for_assets 뒤에 호출한다.")

    # 액션 그래프(OnTick → ConveyorNode)를 만들어 준다. 매 프레임 롤러의 표면 속도를 갱신한다.
    node = create_conveyor_belt(stage, rollers)
    node.GetAttribute("inputs:direction").Set(Gf.Vec3f(1.0, 0.0, 0.0))  # 컨베이어 로컬 +X = 월드 -Y
    node.GetAttribute("inputs:animateTexture").Set(True)  # 벨트 무늬가 흘러가 보이게
    node.GetAttribute("inputs:animateDirection").Set(Gf.Vec2f(0.0, 1.0))
    node.GetAttribute("inputs:animateScale").Set(0.01)
    set_conveyor_running(True, env_path)


def set_conveyor_running(running: bool, env_path: str = ENV_PATH) -> None:
    """컨베이어를 돌리거나 세운다. 그래프 변수 Velocity 하나만 바꾸면 된다."""
    stage = stage_utils.get_current_stage(backend="usd")
    velocity = stage.GetPrimAtPath(f"{env_path}/conveyor/ConveyorBeltGraph").GetAttribute("graph:variable:Velocity")
    if velocity.IsValid():
        velocity.Set(CONVEYOR_SPEED if running else 0.0)
