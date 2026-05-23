from __future__ import annotations


DEFAULT_ROBOT_PREFIX = "iw_hub"
DEFAULT_ROBOT_IDS = tuple(f"{DEFAULT_ROBOT_PREFIX}_{index:02d}" for index in (1, 2))


def default_robot_id(index: int) -> str:
    if index < 1:
        raise ValueError("robot index must be 1 or greater")
    return f"{DEFAULT_ROBOT_PREFIX}_{index:02d}"


def default_odom_topic(index: int) -> str:
    return f"/{default_robot_id(index)}/odom"


def default_cmd_vel_topic(index: int) -> str:
    return f"/{default_robot_id(index)}/cmd_vel"


def default_base_frame(index: int) -> str:
    return f"{default_robot_id(index)}/base_link"
