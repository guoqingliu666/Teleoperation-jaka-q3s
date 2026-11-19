"""Check the current wired route and Servo runtime before hardware launch."""
import argparse
from pathlib import Path
from operator_session import ROOT,validate_teleop_runtime
from hardware_hold_check import validate_site
parser=argparse.ArgumentParser();parser.add_argument('--runtime-config',type=Path,required=True);args=parser.parse_args()
validate_teleop_runtime(args.runtime_config)
validate_site(ROOT)
print('有线路由与 Servo 配置已核对。驱动将在启动时读取控制柜状态、工具、负载和反馈。')
