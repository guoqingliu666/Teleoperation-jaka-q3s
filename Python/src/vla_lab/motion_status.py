"""JAKA ``get_motion_status`` 的严格 Python 结构解释。

字段顺序来自当前 SDK V2.3.1 beta3 的 ``MotionStatus`` 头文件，并由
2026-09-24 静止只读实测确认 Python 返回为 11 元素序列。未知长度、类型或
布尔值一律拒绝，不能把缺失的队列状态猜成空闲。
"""
from __future__ import annotations

from dataclasses import asdict, dataclass


class MotionStatusParseError(ValueError):
    """拒绝未知状态时保留原始 SDK 字段，供失败日志审计。"""

    def __init__(self, message: str, raw) -> None:
        self.raw = tuple(raw) if isinstance(raw, (tuple, list)) else raw
        super().__init__(f"{message}；原始返回={self.raw!r}")


def _binary(value, name: str) -> bool:
    if type(value) not in (bool, int) or value not in (0, 1):
        raise ValueError(f"{name} 不是 0/1：{value!r}")
    return bool(value)


@dataclass(frozen=True)
class MotionStatus:
    motion_line: int
    motion_line_sdk: int
    inpos: bool
    err_add_line: int
    queue: int
    active_queue: int
    queue_full: bool
    paused: bool
    on_limit: bool
    in_estop: bool
    in_collision: bool

    @classmethod
    def parse(cls, value) -> "MotionStatus":
        if not isinstance(value, (tuple, list)) or len(value) != 11:
            raise MotionStatusParseError("MotionStatus 必须是 11 个字段", value)
        if any(type(value[index]) is not int for index in (0, 1, 3, 4, 5)):
            raise MotionStatusParseError("MotionStatus 的编号和队列字段必须是整数", value)
        if value[4] < 0 or value[5] < 0:
            raise MotionStatusParseError("MotionStatus 队列数不能为负", value)
        return cls(
            motion_line=value[0],
            motion_line_sdk=value[1],
            inpos=_binary(value[2], "inpos"),
            err_add_line=value[3],
            queue=value[4],
            active_queue=value[5],
            queue_full=_binary(value[6], "queue_full"),
            paused=_binary(value[7], "paused"),
            on_limit=_binary(value[8], "on_limit"),
            in_estop=_binary(value[9], "in_estop"),
            in_collision=_binary(value[10], "in_collision"),
        )

    def as_dict(self) -> dict:
        return asdict(self)

    def require_idle_safe(self) -> None:
        """真机提交新队列前使用；任一未知/占用/安全状态都会拒绝。"""
        if not self.inpos or self.queue or self.active_queue:
            raise RuntimeError(f"运动队列不是静止空闲：{self.as_dict()}")
        if self.queue_full or self.paused or self.on_limit or self.in_estop or self.in_collision:
            raise RuntimeError(f"运动队列状态禁止下发：{self.as_dict()}")
