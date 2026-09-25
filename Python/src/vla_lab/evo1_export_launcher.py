"""Dependency-free Windows launcher for the WSL Evo-1 episode exporter."""

from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path
from typing import Any


DEFAULT_FPS = 5.0


def launch_wsl_export(
    source_episode: str | Path,
    output_root: str,
    *,
    fps: float = DEFAULT_FPS,
    log_path: str | Path | None = None,
) -> subprocess.Popen[bytes]:
    """Start an offline export in WSL's existing ``Evo1`` environment.

    This module intentionally has no NumPy, OpenCV, Pandas, or PyArrow
    dependency, because it is imported by the Windows hardware-control GUI.
    The actual conversion runs only inside WSL after recording has completed.
    """

    source_episode = Path(source_episode).resolve()
    source_root = Path(__file__).resolve().parents[1]

    def to_wsl_path(path: Path) -> str:
        """Convert an absolute Windows path without a separate WSL call.

        The GUI runs in a Windows Python runtime.  Calling ``wslpath`` in a
        preliminary process can fail even though the later WSL command is
        usable (for example, while Windows is still bringing up the distro).
        The project and recorded episodes are on a local drive, whose WSL
        mount convention is deterministic.
        """

        value = str(path)
        if os.name != "nt":
            return value
        drive, tail = os.path.splitdrive(value)
        if not drive:
            raise ValueError(f"Expected an absolute Windows path, got: {value}")
        normalized_tail = tail.lstrip("\\/").replace("\\", "/")
        return f"/mnt/{drive[0].lower()}/{normalized_tail}"

    source_wsl = to_wsl_path(source_episode)
    package_wsl = to_wsl_path(source_root)
    output_wsl = output_root
    if os.name == "nt" and len(output_root) >= 2 and output_root[1] == ":":
        output_wsl = to_wsl_path(Path(output_root))
    command = " && ".join(
        [
            "source /home/yufeng/miniconda3/etc/profile.d/conda.sh",
            "conda run -n Evo1 env "
            f"PYTHONPATH={shlex.quote(package_wsl)} "
            "python -m vla_lab.evo1_export "
            f"--episode {shlex.quote(source_wsl)} "
            f"--output {shlex.quote(output_wsl)} --fps {float(fps)}",
        ]
    )
    stdout: Any = subprocess.DEVNULL
    if log_path is not None:
        log_file = Path(log_path)
        log_file.parent.mkdir(parents=True, exist_ok=True)
        stdout = log_file.open("wb")
    process = subprocess.Popen(
        ["wsl.exe", "bash", "-lc", command],
        stdout=stdout,
        stderr=subprocess.STDOUT,
    )
    if log_path is not None:
        stdout.close()
    return process
