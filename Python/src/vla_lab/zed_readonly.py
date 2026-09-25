"""Read-only ZED Mini RGB or RGB-D adapter used by the recorder."""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Any

from .monitor_config import ZedMonitorConfig
from .monitor_types import CameraFrame


ZED_BIN = (
    Path(os.environ.get("ZED_SDK_ROOT_DIR", r"C:\Program Files (x86)\ZED SDK"))
    / "bin"
)
_DLL_DIRECTORY: Any = None


def _load_zed() -> Any:
    """Make ZED SDK DLLs visible before importing ``pyzed.sl`` on Windows."""

    global _DLL_DIRECTORY
    if os.name == "nt" and _DLL_DIRECTORY is None:
        if not ZED_BIN.exists():
            raise RuntimeError(f"ZED SDK bin directory not found: {ZED_BIN}")
        _DLL_DIRECTORY = os.add_dll_directory(str(ZED_BIN))
    try:
        import pyzed.sl as sl
    except ModuleNotFoundError:
        # The ACT environment is intentionally separate from the stable ZED/JAKA
        # runtime.  For combined read-only shadow inference it may reuse only the
        # already validated pyzed package directory, appended after ACT's own
        # site-packages so torch/numpy/opencv are never replaced.
        fallback = os.environ.get("JZVLA_PYZED_SITE_PACKAGES", "").strip()
        if not fallback:
            raise
        fallback_path = Path(fallback).resolve()
        if not (fallback_path / "pyzed").is_dir():
            raise RuntimeError(f"pyzed fallback directory is invalid: {fallback_path}")
        if str(fallback_path) not in sys.path:
            sys.path.append(str(fallback_path))
        import pyzed.sl as sl

    return sl


class ZedLeftCamera:
    """Open the camera and retrieve configured RGB(/D); no settings are written."""

    def __init__(self, config: ZedMonitorConfig) -> None:
        self.config = config
        self._sl: Any = None
        self._camera: Any = None
        self._image: Any = None
        self._depth: Any = None
        self._frame_id = 0
        self._serial_number: int | None = None

    def open(self) -> None:
        sl = _load_zed()
        camera = sl.Camera()
        init = sl.InitParameters()
        try:
            init.camera_resolution = getattr(sl.RESOLUTION, self.config.resolution)
        except AttributeError as error:
            raise ValueError(
                f"Unsupported ZED resolution: {self.config.resolution}"
            ) from error
        init.camera_fps = int(self.config.fps)
        try:
            init.depth_mode = getattr(sl.DEPTH_MODE, self.config.depth_mode)
        except AttributeError as error:
            raise ValueError(
                f"Unsupported ZED depth mode: {self.config.depth_mode}"
            ) from error
        if self.config.serial_number is not None:
            init.set_from_serial_number(int(self.config.serial_number))
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            camera.close()
            raise RuntimeError(
                f"ZED open failed: {status}. Close Depth Viewer/Explorer/Windows Camera."
            )
        self._sl = sl
        self._camera = camera
        self._image = sl.Mat()
        self._depth = sl.Mat() if self.config.capture_depth else None
        self._serial_number = int(camera.get_camera_information().serial_number)
        self._frame_id = 0

    def read_frame(self) -> CameraFrame:
        if (
            self._camera is None
            or self._image is None
            or self._sl is None
        ):
            raise RuntimeError("ZED camera is not open")
        if self._camera.grab() != self._sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("ZED grab failed")
        self._camera.retrieve_image(self._image, self._sl.VIEW.LEFT)
        if self._depth is not None:
            self._camera.retrieve_measure(self._depth, self._sl.MEASURE.DEPTH)
        # ZED returns a 4-channel CPU array; OpenCV/Pillow only need BGR here.
        image_bgr = self._image.get_data()[:, :, :3].copy()
        depth_mm = self._depth.get_data().copy() if self._depth is not None else None
        frame = CameraFrame(
            timestamp_ns=time.time_ns(),
            frame_id=self._frame_id,
            image_bgr=image_bgr,
            depth_mm=depth_mm,
            camera_name="zed_mini_left",
            serial_number=self._serial_number,
        )
        self._frame_id += 1
        return frame

    def close(self) -> None:
        if self._camera is not None:
            self._camera.close()
        self._camera = None
        self._image = None
        self._depth = None
