"""手工验收当前的 Windows Player：接收真实 XR 帧，不注入模拟数据。

本文件会启动 VR 程序并等待头显数据，不属于可在无人值守 CI 中运行的单元测试。
旧版 Player 已归档；这里必须始终检查①实际使用的构建。
"""
from pathlib import Path
import ctypes
from ctypes import wintypes
import json
import socket
import subprocess
import time


def run():
    root = Path(__file__).resolve().parents[2]
    player = root / "Player_通信修复" / "QuestPosePreview.exe"
    assert player.is_file()
    packets = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(.5)
        port = receiver.getsockname()[1]
        process = subprocess.Popen([str(player), "-pose-port", str(port), "-logFile", str(root / "Logs" / "PlayerSmoke.log")], cwd=root)
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                try:
                    data, _ = receiver.recvfrom(65535)
                    packets.append(json.loads(data))
                except socket.timeout:
                    if process.poll() is not None:
                        break
            report = {"packets": len(packets), "source": "real XR input (not synthetic)",
                      "valid_counts": {key: sum(bool(p.get(key, {}).get("pose_valid")) for p in packets) for key in ("head", "left", "right")},
                      "last": packets[-1] if packets else None}
            (root / "Validation" / "player_real_xr.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
            print(json.dumps({k:v for k,v in report.items() if k != "last"}))
            assert packets, "No UDP from standalone player; inspect PlayerSmoke.log"
        finally:
            # 只关闭本脚本创建的预览窗口，绝不关闭用户的 Unity 或 Meta 应用。
            callback_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
            @callback_type
            def close_window(hwnd, _):
                pid = wintypes.DWORD()
                ctypes.windll.user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
                if pid.value == process.pid:
                    ctypes.windll.user32.PostMessageW(hwnd, 0x0010, 0, 0)
                return True
            ctypes.windll.user32.EnumWindows(close_window, 0)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log = (root / "Logs" / "PlayerSmoke.log").read_text(encoding="utf-8", errors="replace")
        assert "Exception:" not in log, "Runtime exception in PlayerSmoke.log"


if __name__ == "__main__":
    run()
