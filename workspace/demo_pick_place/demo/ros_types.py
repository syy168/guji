from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class RosTypes:
    Movej: Any
    Movel: Any
    Gripperpick: Any
    Sixforce: Any


def import_rm_ros_types() -> RosTypes:
    # 延迟导入：保证在非 ROS 环境下导入模块时不立即报错。
    from rm_ros_interfaces.msg import Gripperpick, Movej, Movel, Sixforce

    return RosTypes(Movej=Movej, Movel=Movel, Gripperpick=Gripperpick, Sixforce=Sixforce)
