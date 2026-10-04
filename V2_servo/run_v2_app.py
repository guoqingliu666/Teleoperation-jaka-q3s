"""V2唯一用户入口。主窗口不拥有SDK；动作必须明确选择。"""
import os,sys,multiprocessing as mp
from pathlib import Path
ROOT=Path(__file__).resolve().parent
os.environ['TEMP']='D:/ChatGPT/Temp';os.environ['TMP']=os.environ['TEMP']
os.environ['PYTHONDONTWRITEBYTECODE']='1';sys.dont_write_bytecode=True
for key in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[key]='1'
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'.deps'),str(ROOT)]
os.environ['PYTHONPATH']=os.pathsep.join([str(ROOT/'src'),str(ROOT/'.deps'),str(ROOT)])
if __name__=='__main__':
    mp.freeze_support()
    from v2_servo.launcher import main
    raise SystemExit(main())
