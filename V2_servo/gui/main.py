"""在 PyCharm 直接运行，或 conda activate jaka 后 python gui/main.py。"""
import os,sys,multiprocessing as mp
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
os.environ['TEMP']='D:/ChatGPT/Temp'; os.environ['TMP']='D:/ChatGPT/Temp'
os.environ['MPLCONFIGDIR']='D:/ChatGPT/Cache/V2_servo/matplotlib'
os.environ['PYTHONDONTWRITEBYTECODE']='1'; sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'src'))
if __name__=='__main__':
    mp.freeze_support()
    from v2_servo.gui.main import main
    raise SystemExit(main())
