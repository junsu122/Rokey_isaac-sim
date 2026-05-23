from __future__ import annotations

from typing import Optional

from smart_factory.robot1_stack_sequence import main as _run_stack_sequence


def main(args: Optional[list[str]] = None) -> None:
    _run_stack_sequence(
        args,
        default_robot_index=2,
        default_stack_target="STACK_2",
        default_shelf_storage_target="SHELF_STORAGE_2",
        default_unload_target="UNLOAD_2",
        default_wait_target="WAIT_3",
        default_status_topic="/smart_factory/robot2_stack_sequence_status",
    )


if __name__ == "__main__":
    main()
