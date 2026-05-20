"""
main_isaac/world_setup.py
=========================
창고 씬 공통 설정 (맵 로드 + 조명).
"""
from isaacsim.core.utils.prims import define_prim
from pxr import Sdf, Gf

from robot_config import WAREHOUSE_USD


def setup_warehouse(world) -> None:
    """창고 USD 로드 + 원거리 조명 추가."""
    # 창고 맵
    warehouse = define_prim("/World/Warehouse", "Xform")
    warehouse.GetReferences().AddReference(WAREHOUSE_USD)

    # 전체 조명
    light = define_prim("/World/DistantLight", "DistantLight")
    light.CreateAttribute("intensity", Sdf.ValueTypeNames.Float).Set(3000.0)
    light.CreateAttribute("angle",     Sdf.ValueTypeNames.Float).Set(0.53)

    print(f"[WorldSetup] 창고 맵 로드 완료: {WAREHOUSE_USD}")
