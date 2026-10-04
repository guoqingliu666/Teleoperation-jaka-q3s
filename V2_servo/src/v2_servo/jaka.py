"""JAKA V2.3.1_beta3 接口适配。只使用核查过的名字，不自动上电、使能、复位。

独立适配器可在 FakeRC 测试，也可由已验收的部署驱动调用。
现有工程未提供经验证的并发反馈/控制器看门狗/碰撞部署，CLI 不开放真机运动。
"""
import os, sys, time
from pathlib import Path
import numpy as np
from .configuration import vector
from .safety import Feedback

def checked(ret,name):
    if not isinstance(ret,(tuple,list)) or not ret or type(ret[0]) is not int or ret[0]!=0:
        raise RuntimeError(f'{name} failed: {ret!r}')
    return ret[1] if len(ret)>1 else None

def checked_bool(ret,name):
    v=checked(ret,name)
    if type(v) not in (int,bool) or v not in (0,1): raise RuntimeError(f'{name}: invalid boolean {v!r}')
    return bool(v)

def load_sdk(directory):
    directory=Path(directory).resolve()
    if not (directory/'jkrc.pyd').is_file() and not (directory/'jkrc.so').is_file(): raise FileNotFoundError('matching jkrc binary missing')
    # DLL 搜索句柄必须保持存活。只加载模块；此处不实例化 RC，不登录。
    if hasattr(os,'add_dll_directory'): DLL_HANDLES.append(os.add_dll_directory(str(directory)))
    sys.path.insert(0,str(directory))
    import jkrc
    return jkrc

DLL_HANDLES=[]

def decode_status(ret,started,sequence):
    s=checked(ret,'get_robot_status')
    if not isinstance(s,(list,tuple)) or len(s) not in (24,25): raise RuntimeError('unknown status schema; fail closed')
    def flag(i):
        if type(s[i]) not in (int,bool) or s[i] not in (0,1): raise RuntimeError(f'invalid status[{i}]')
        return bool(s[i])
    if type(s[0]) is not int or not flag(22): raise RuntimeError('invalid error/socket state')
    # 普通状态中的电流不是关节力矩；绝不冒充 Nm。
    return Feedback(vector(s[19]),started,sequence,enabled=flag(3),powered=flag(2),
                    estop=flag(23),collision=flag(5),soft_limit=flag(7),error=s[0],torque=None)

class JakaServo:
    def __init__(self,rc,cfg,authorization):
        self.rc=rc; self.cfg=cfg; self.authorization=authorization
        self.entered=False; self.enter_attempted=False; self.sent=0; self.filter_set=False
        self.last_send=None
    def start(self):
        self.authorization('start')
        if not self.cfg['filters']['enabled']: raise RuntimeError('live filtering required')
        if checked_bool(self.rc.is_in_servomove(),'is_in_servomove'): raise RuntimeError('controller already in servo: ownership unknown')
        # 真机默认保留控制柜配置；指令低通＋Ruckig由软件链负责。
        # 仅显式配置该策略才在退出SERVO时设置LPF；setter成功不是独立读回。
        policy=self.cfg['filters'].get('controller_lpf_policy','preserve')
        if policy=='set_before_servo':
            checked(self.rc.servo_move_use_joint_LPF(self.cfg['filters']['controller_lpf_hz']),'servo_move_use_joint_LPF')
        elif policy!='preserve': raise RuntimeError('invalid controller filter policy')
        self.filter_set=True
        self.authorization('enable')
        self.enter_attempted=True
        checked(self.rc.servo_move_enable(True),'servo_move_enable')
        if not checked_bool(self.rc.is_in_servomove(),'is_in_servomove'): raise RuntimeError('servo enable unconfirmed')
        self.entered=True
    def send(self,motion):
        self.authorization('send')
        if not self.entered or not self.filter_set: raise RuntimeError('servo/filter not ready')
        q=vector(motion.q)
        # ABS=0，关节 rad，step_num=1 即 8 ms；不使用 V3 新增 DO 参数。
        start=time.perf_counter()
        self.last_send={'started_s':start,'returned_s':None,'elapsed_s':None,
                        'native_return':None,'accepted':False,
                        'acceptance_scope':getattr(self.rc,'command_acceptance_scope','sdk_normal_return')}
        try:
            ret=self.rc.servo_j(q.tolist(),0,1)
            self.last_send['native_return']=ret
        finally:
            end=time.perf_counter()
            self.last_send.update(returned_s=end,elapsed_s=end-start)
        checked(ret,'servo_j')
        self.sent+=1;self.last_send['accepted']=True
        if len(ret)>1:
            depth=ret[1];self.last_send['controller_queue_depth']=depth
            # 本地beta3扩展接口可返回队列长度；低延迟候选只允许最多2帧积压。
            # 目标已经受理后才有此反馈，异常必须保留该次发送并停止。
            if type(depth) is not int or not 0<=depth<=2:
                raise RuntimeError('servo_j queue depth invalid/exceeds 2: '+repr(depth))
        elapsed=self.last_send['elapsed_s']
        if elapsed>self.cfg['safety']['max_call_s']:
            raise RuntimeError(f'SDK call overrun: {elapsed*1000:.3f} ms > '
                               f'{self.cfg["safety"]["max_call_s"]*1000:.3f} ms; external watchdog required')
        return elapsed
    def stop(self):
        errors=[]
        if not self.enter_attempted: return {'servo_exited':None,'physical_stop_confirmed':False,'errors':['servo not owned']}
        # 每个清理步骤独立尝试；失败不掩盖前一个错误。绝不关闭滤波或自动清报警。
        for name,args in [('motion_abort',()),('servo_move_enable',(False,))]:
            try: checked(getattr(self.rc,name)(*args),name)
            except Exception as e: errors.append(str(e))
        exited=False
        try: exited=not checked_bool(self.rc.is_in_servomove(),'is_in_servomove')
        except Exception as e: errors.append(str(e))
        self.entered=False
        return {'servo_exited':exited,'physical_stop_confirmed':False,'errors':errors}

def readonly_probe(directory,host,log_dir,samples=20):
    sdk=load_sdk(directory); rc=sdk.RC(host); rows=[]; logged=False
    log_path_result=None
    try:
        # 部分 SDK/旧控制器不接受日志路径接口。只读连接仍可诊断，显式记录结果。
        # 进程 cwd 与 task TEMP 已在 D；不声称能控制厂商/操作系统全部内部日志。
        log_path_result=rc.set_SDK_filepath(str(log_dir)+os.sep)
        checked(rc.login(),'login'); logged=True
        for i in range(samples):
            start=time.perf_counter(); ret=rc.get_robot_status(); finish=time.perf_counter()
            f=decode_status(ret,start,i)
            rows.append({'sequence':i,'query_start':start,'query_end':finish,'cost_s':finish-start,
                         'q_rad':f.q.tolist(),'estop':f.estop,'collision':f.collision,'error':f.error})
        return {'scope':'live_readonly','movement_commands_sent':0,'sdk_log_path_result':log_path_result,'rows':rows}
    finally:
        if logged: checked(rc.logout(),'logout')
