import argparse, json, sys, time, os
from pathlib import Path
from .configuration import ROOT,load
from .experiments import run_all,write_json

def main(argv=None):
    p=argparse.ArgumentParser(description='V2 伺服控制版本：默认离线；真机运动尚未完成现场放行。')
    p.add_argument('command',nargs='?',default='demo',choices=['demo','test-matrix','sdk-audit','readonly','live','stop'])
    p.add_argument('--quick',action='store_true'); p.add_argument('--output',type=Path)
    p.add_argument('--sdk-dir',type=Path); p.add_argument('--host'); p.add_argument('--samples',type=int,default=20)
    args=p.parse_args(argv)
    if args.command=='stop':
        (ROOT/'logs/STOP.request').write_text(str(time.time()),encoding='utf-8')
        print('软件停止请求已写入；真机物理急停独立于该文件。'); return 0
    if args.command=='live':
        print('BLOCKED: V2 尚缺场景/安装标定、独立看门狗、发送与反馈时序验收、滤波独立读回和事故硬件检查。既有 TCP/负载只读记录不解锁运动。未加载 SDK，未连接，运动命令 0。')
        return 90
    if args.command=='sdk-audit':
        if not args.sdk_dir: p.error('--sdk-dir required')
        from .jaka import load_sdk
        sdk=load_sdk(args.sdk_dir)
        names=['login','logout','set_SDK_filepath','get_robot_status','get_actual_joint_position','get_actual_tcp_position',
               'kine_inverse','servo_move_enable','servo_j','is_in_servomove','servo_move_use_joint_LPF','motion_abort']
        result={'scope':'module_only_no_RC_created','binary':str(sdk.__file__),'methods':{n:hasattr(sdk.RC,n) for n in names}}
        if args.output: write_json(args.output,result)
        print(json.dumps(result,ensure_ascii=False,indent=2)); return 0 if all(result['methods'].values()) else 2
    if args.command=='readonly':
        if not args.sdk_dir or not args.host: p.error('--sdk-dir and --host required; no default robot address')
        if not 1<=args.samples<=1000: p.error('samples must be 1..1000')
        from .jaka import readonly_probe
        out=args.output or ROOT/'logs'/('readonly_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
        result=readonly_probe(args.sdk_dir,args.host,ROOT/'logs',args.samples); write_json(out,result)
        print(f'只读结果: {out}; 运动命令 0'); return 0
    out=args.output or ROOT/'data'/('run_'+time.strftime('%Y%m%d_%H%M%S'))
    if args.command=='demo':
        from .demo import run_demo
        return run_demo(out)
    return 0 if run_all(out,args.quick)['software_checks_pass'] else 2
