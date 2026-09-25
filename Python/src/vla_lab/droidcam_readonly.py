"""Read-only DroidCam source addressed by its Windows DirectShow name."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

from .monitor_types import CameraFrame


def _load_pyav() -> Any:
    """Load PyAV without replacing the hardware runtime's core packages."""

    try:
        import av

        return av
    except ModuleNotFoundError:
        act_site_packages = (
            Path(__file__).resolve().parents[2]
            / ".venv-act"
            / "Lib"
            / "site-packages"
        )
        if act_site_packages.is_dir() and str(act_site_packages) not in sys.path:
            sys.path.append(str(act_site_packages))
        try:
            import av

            return av
        except ModuleNotFoundError as error:
            raise RuntimeError(
                "PyAV is unavailable; run setup_act_environment.cmd first"
            ) from error


class DroidCamReadOnlyCamera:
    """Decode the latest Redmi/DroidCam RGB stream without saving or control."""

    def __init__(self, config: object, *, av_module: Any | None = None) -> None:
        self.device_name = str(config.device_name)
        self.width = int(config.width)
        self.height = int(config.height)
        self.fps = int(config.fps)
        self._av = av_module
        self._container: Any | None = None
        self._frames: Any | None = None
        self._frame_id = 0

    def open(self) -> None:
        if self._container is not None:
            return
        if self._av is None:
            self._av = _load_pyav()
        self._container = self._av.open(
            f"video={self.device_name}",
            format="dshow",
            options={
                "video_size": f"{self.width}x{self.height}",
                "framerate": str(self.fps),
            },
        )
        self._frames = self._container.decode(video=0)

    def read_frame(self) -> CameraFrame:
        if self._frames is None:
            raise RuntimeError("DroidCam is not open")
        try:
            frame = next(self._frames)
        except StopIteration as error:
            raise RuntimeError("DroidCam video stream ended") from error
        image_bgr = frame.to_ndarray(format="bgr24")
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise RuntimeError(f"invalid DroidCam frame shape: {image_bgr.shape}")
        result = CameraFrame(
            timestamp_ns=time.time_ns(),
            frame_id=self._frame_id,
            image_bgr=image_bgr,
            depth_mm=None,
            camera_name="droidcam_global_rgb",
            serial_number=None,
        )
        self._frame_id += 1
        return result

    def close(self) -> None:
        container, self._container = self._container, None
        self._frames = None
        if container is not None:
            container.close()
