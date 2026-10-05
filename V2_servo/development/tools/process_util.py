"""Windows test-process helpers; only close the process created by the caller."""
import ctypes
import subprocess


def close_own_player(process):
    user = ctypes.windll.user32
    callback_type = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)

    def close_window(hwnd, _):
        pid = ctypes.c_ulong()
        user.GetWindowThreadProcessId(ctypes.c_void_p(hwnd), ctypes.byref(pid))
        if pid.value == process.pid:
            user.PostMessageW(ctypes.c_void_p(hwnd), 0x0010, 0, 0)
        return True

    callback = callback_type(close_window)
    user.EnumWindows(callback, 0)
    try:
        return process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        raise RuntimeError('Own Player did not close; close its window manually. No process was killed.')
