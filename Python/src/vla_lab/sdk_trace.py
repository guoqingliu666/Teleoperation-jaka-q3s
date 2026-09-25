"""SDK边界诊断：调用前落盘、调用后计时；不并发调用、不改控制器参数。

150ms是本程序对返回数据的时效门槛，不是SDK网络超时设置。
阻塞中的原生调用无法靠此包装器中断；若返回太晚，锁住后续查询/运动，
只允许原线程尝试停止、查到位和注销，绝不补发积压运动。
"""
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import time


class TraceFile:
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.logger = logging.Logger(str(path))
        self.handler = RotatingFileHandler(path, maxBytes=16*1024*1024, backupCount=3, encoding="utf-8")
        self.handler.setFormatter(logging.Formatter("%(message)s"))
        self.logger.addHandler(self.handler)

    def write(self, event):
        self.logger.info(json.dumps(event, ensure_ascii=False))

    def close(self):
        self.handler.close()
        self.logger.removeHandler(self.handler)


class TracedSdk:
    CLEANUP = frozenset(("motion_abort", "is_in_pos", "logout"))

    def __init__(self, robot, trace, progress=None, *, clock=time.perf_counter, max_call_s=.15):
        self.raw, self.trace, self.progress = robot, trace, progress
        self.clock, self.max_call_s = clock, max_call_s
        self.serial = 0
        self.fault = None
        self.cache = {}

    def __getattr__(self, name):
        method = getattr(self.raw, name)
        if not callable(method): return method
        if name not in self.cache:
            def call(*args, **kwargs):
                if self.fault and name not in self.CLEANUP:
                    raise RuntimeError(f"SDK已锁定：{self.fault}；拒绝后续{name}")
                self.serial += 1
                call_id = self.serial
                event = dict(state="sdk_begin", call_id=call_id, method=name,
                             time_ns=time.time_ns())
                self.trace.write(event)
                if self.progress: self.progress(event)
                started = self.clock()
                result, error = None, None
                try:
                    result = method(*args, **kwargs)
                except Exception as exc:
                    error = repr(exc)
                    self.fault = f"{name}: {error}"
                    raise
                finally:
                    elapsed = self.clock()-started
                    code = result[0] if isinstance(result, tuple) and result else None
                    late = name not in ("login", "logout") and elapsed > self.max_call_s
                    if late or code == -3:
                        self.fault = f"{name} 耗时{elapsed*1000:.1f}ms，返回{code}"
                    event = dict(state="sdk_end", call_id=call_id, method=name,
                                 time_ns=time.time_ns(), elapsed_ms=elapsed*1000,
                                 code=code, exception=error, late=late)
                    self.trace.write(event)
                    if self.progress: self.progress(event)
                if late and name not in self.CLEANUP:
                    raise TimeoutError(f"SDK返回过晚：{self.fault}；不使用迟到结果")
                return result
            self.cache[name] = call
        return self.cache[name]
