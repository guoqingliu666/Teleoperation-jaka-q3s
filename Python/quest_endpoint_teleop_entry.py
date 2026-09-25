"""ASCII 文件名入口，避免 Windows CMD 在 UTF-8 中文命令行上截断参数。"""

from pathlib import Path
import runpy


if __name__ == "__main__":
    runpy.run_path(
        str(Path(__file__).with_name("连续采样位置遥操作.py")),
        run_name="__main__",
    )
