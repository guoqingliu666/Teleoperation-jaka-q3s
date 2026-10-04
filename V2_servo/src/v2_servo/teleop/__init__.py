"""独立输入、周期控制与驱动。可选二进制依赖限定在项目内。"""
import sys
from pathlib import Path
deps=Path(__file__).resolve().parents[3]/'.deps'
if deps.is_dir() and str(deps) not in sys.path: sys.path.insert(0,str(deps))
