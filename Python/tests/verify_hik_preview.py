"""无实物取流测试 + 真实 SDK 枚举/Tk 面板验收。绝不调用实物 OpenDevice。"""
from pathlib import Path
import ctypes as ct
import json
import queue
import sys
import threading
import tkinter as tk
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab import hik_camera_preview as hik

OUT = Path(__file__).resolve().parents[2] / "Validation"


class WorkerTests(unittest.TestCase):
    def test_identity_not_index(self):
        items = [dict(serial="other", interface="1", index=0), dict(serial="target", interface="1", index=5)]
        self.assertEqual(hik.select_device(items, dict(serial="target", interface="1"))["index"], 5)
        with self.assertRaises(RuntimeError):
            hik.select_device(items, dict(serial="target", interface="2"))
        with self.assertRaises(RuntimeError):
            hik.select_device(items + [items[1]], dict(serial="target", interface="1"))

    def test_worker_lifecycle_and_failures(self):
        # 仅借用官方结构体类型；所有 SDK 函数都替换成 fake，绝不打开实物。
        sdk = hik.load_sdk()
        info = sdk.MV_CC_DEVICE_INFO()
        pointer = ct.pointer(info)
        listing = type("Listing", (), {"pDeviceInfo": [pointer]})()
        selected = dict(serial="fake", interface="1", index=0, name="SYNTHETIC", ip="0.0.0.0")
        for fail, expected in [(None, "done"), ("open", "error"), ("trigger", "error"), ("convert", "error")]:
            calls = []
            stop = threading.Event()
            image_buffer = (ct.c_ubyte * 12)(*range(12))
            class FakeCamera:
                @staticmethod
                def MV_CC_Initialize(): return 0
                @staticmethod
                def MV_CC_Finalize(): calls.append("finalize"); return 0
                def MV_CC_CreateHandle(self, _): calls.append("create"); return 0
                def MV_CC_OpenDevice(self, mode, _):
                    self.mode = mode
                    calls.append("open")
                    return 1 if fail == "open" else 0
                def MV_CC_GetEnumValue(self, name, target):
                    target.nCurValue = 1 if fail == "trigger" else 0
                    return 0
                def MV_CC_SetImageNodeNum(self, _): return 0
                def MV_CC_SetGrabStrategy(self, _): return 0
                def MV_CC_StartGrabbing(self): calls.append("start"); return 0
                def MV_CC_GetImageBuffer(self, frame, _):
                    frame.pBufAddr = ct.cast(image_buffer, ct.POINTER(ct.c_ubyte))
                    frame.stFrameInfo.nWidth = 2
                    frame.stFrameInfo.nHeight = 2
                    frame.stFrameInfo.nFrameLen = 12
                    frame.stFrameInfo.nFrameNum = 1
                    stop.set()
                    return 0
                def MV_CC_ConvertPixelTypeEx(self, param):
                    ct.memmove(param.pDstBuffer, image_buffer, 12)
                    param.nDstLen = 12
                    return 1 if fail == "convert" else 0
                def MV_CC_FreeImageBuffer(self, _): calls.append("free"); return 0
                def MV_CC_StopGrabbing(self): calls.append("stop"); return 0
                def MV_CC_CloseDevice(self): calls.append("close"); return 0
                def MV_CC_DestroyHandle(self): calls.append("destroy"); return 0
            class FrameQueue(queue.Queue):
                def cancel_join_thread(self): pass
            events, frames, capture_requests = queue.Queue(), FrameQueue(1), queue.Queue()
            capture_file = (Path(__file__).resolve().parents[1]
                            / "datasets" / "hardware_observations" / "_verify_full_frame.jpg")
            capture_file.unlink(missing_ok=True)
            if fail is None:
                capture_requests.put(("test-token", str(capture_file)))
            with patch.object(hik, "load_sdk", return_value=sdk), patch.object(sdk, "MvCamera", FakeCamera), \
                 patch.object(hik, "enumerate_devices", return_value=(listing, [selected])):
                hik.camera_worker("preview", selected, events, frames, stop, capture_requests)
            messages = list(events.queue)
            self.assertIn(expected, [m[0] for m in messages], (fail, messages))
            self.assertEqual(calls[-2:], ["destroy", "finalize"])
            self.assertEqual("close" in calls, fail != "open")
            self.assertEqual("free" in calls, fail in (None, "convert"))
            if fail is None:
                self.assertEqual(frames.get_nowait()["rgb"], bytes(range(12)))
                self.assertTrue(capture_file.is_file())
                self.assertIn("capture", [m[0] for m in messages])
                from PIL import Image
                with Image.open(capture_file) as captured:
                    self.assertEqual(captured.size, (2, 2))
                capture_file.unlink()
            else:
                self.assertTrue(frames.empty())


def verify_panel():
    root = tk.Tk()
    root.title("相机面板自动验收：只枚举，不连接")
    root.geometry("1100x780")
    root.attributes("-topmost", True)
    root.lift()
    panel = hik.HikPreviewPanel(root)
    panel.pack(fill="both", expand=True)
    failures = []
    started = time.monotonic()
    def inspect():
        try:
            if panel.process is not None:
                if time.monotonic() - started > 20:
                    raise AssertionError("枚举进程未按时结束")
                root.after(100, inspect)
                return
            assert panel.devices, panel.status.get()
            assert panel.last_sample is None
            (OUT / "hik_devices.json").write_text(json.dumps(panel.devices, ensure_ascii=False, indent=2), encoding="utf-8")
            # 明确标记的合成色块，验证真正 Tk 图像显示；不是相机现场画面。
            from PIL import Image, ImageDraw, ImageGrab
            image = Image.new("RGB", (640, 480), "#123456")
            draw = ImageDraw.Draw(image)
            draw.rectangle((20, 20, 300, 450), fill="#ef7425")
            draw.rectangle((340, 20, 620, 450), fill="#19bce3")
            sample = dict(size=image.size, rgb=image.tobytes())
            panel._show_sample(sample)
            panel.status.set(f"SYNTHETIC DISPLAY TEST / NOT CAMERA IMAGE | SDK found {len(panel.devices)} devices; none opened")
            root.update()
            assert panel.photo is not None
            ImageGrab.grab(bbox=(root.winfo_rootx(), root.winfo_rooty(), root.winfo_rootx()+root.winfo_width(), root.winfo_rooty()+root.winfo_height())).save(OUT / "hik_panel_synthetic.png")
            print("HIK_TK_ENUM_PASS", len(panel.devices), "devices; synthetic RGB rendered; no real camera opened")
        except Exception as error:
            failures.append(str(error))
        finally:
            if panel.process is None or failures:
                panel.close()
                root.destroy()
    root.after(200, panel.discover)
    root.after(1500, inspect)
    root.mainloop()
    if failures:
        raise AssertionError(failures)


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(WorkerTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():
        sys.exit(1)
    verify_panel()
