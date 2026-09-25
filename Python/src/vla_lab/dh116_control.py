"""DH116 右手独立控制通道：Grip 许可，Trigger 连续开合。

为什么另开进程：EtherCAT/DLL 同步调用不应阻塞 Tk 或 JAKA 控制进程。GUI 只写最新
心跳/目标，不堆积一串过时开合命令。子进程独立检查心跳、反馈、报警和运动授权。
连接不自动使能、不回零、不闭合；需操作者另点“准备手控制”，并先释放 Trigger。
本模块不是安全 PLC；Windows 调度/链路故障无法靠 Python 保证物理急停时间。
"""
from __future__ import annotations

import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
THIRD_PARTY = ROOT / 'Python/third_party'
HEARTBEAT_S = 0.25
FEEDBACK_S = 0.20
ACTIVE_DEGREES = (60., 30., 80., 80., 80., 80.)


def runtime_paths():
    """定位随项目保存的依赖，不修改用户全局 conda 环境。"""
    for path in (THIRD_PARTY, THIRD_PARTY / 'dh116_runtime'):
        if str(path) not in sys.path: sys.path.insert(0, str(path))


def adapters():
    """只枚举本机适配器，不打开网卡、不向设备发送报文。"""
    runtime_paths()
    import pysoem
    def decode(v): return v.decode(errors='replace') if isinstance(v, bytes) else str(v)
    return [(decode(a.name), decode(a.desc)) for a in pysoem.find_adapters()]


def validate_limits(max_percent, speed_percent_s, current_permille):
    values = tuple(float(v) for v in (max_percent, speed_percent_s, current_permille))
    if not all(math.isfinite(v) for v in values): raise ValueError('手参数必须是有限数字')
    # 首次接入只允许小行程验收；不是灵巧手额定能力，也不是已验证的夹持力。
    if not 5 <= values[0] <= 50: raise ValueError('手闭合上限应为 5—50% 行程')
    if not 1 <= values[1] <= 20: raise ValueError('手速度应为 1—20% 行程/秒')
    if not 50 <= values[2] <= 300: raise ValueError('电流上限应为 50—300‰，不是 mA')
    return values


class TriggerPolicy:
    """纯函数式控制状态，可离线测：新一段 Grip 必须先看到 Trigger 松开。

    实物角度只用于显示，行程使用厂家 0..10000 当量。拇指侧摆(1号)保持连接时的
    当前位置，避免尚未验收的侧摆；五路弯曲(2..6号)同步开合，每次限制增量。
    """
    def __init__(self):
        self.ready = False
        self.target = None

    def reset(self):
        self.ready = False; self.target = None

    def step(self, *, permit, trigger, positions, dt, max_percent, speed_percent_s):
        if not permit:
            self.reset(); return None
        if not math.isfinite(trigger) or not 0 <= trigger <= 1:
            self.reset(); raise ValueError('Trigger 必须为 0—1')
        if len(positions) != 6 or not all(math.isfinite(p) and -100 <= p <= 10100 for p in positions):
            self.reset(); raise ValueError('手反馈行程无效')
        if not self.ready:
            if trigger > 0.10: return None
            self.ready = True; self.target = list(positions)
        desired = trigger * max_percent * 100
        # 软件目标不超前实测超过一个很小步长，通信/电机追踪落后时不累积“追赶债务”。
        step = speed_percent_s * 100 * min(max(dt, 0), 0.05)
        self.target = [positions[0]] + [max(0., min(10000., p + max(-step, min(step, desired-p)))) for p in positions[1:]]
        return self.target


class EthercatHand:
    """精确选择网卡与 ESI 身份，只接受一只 DH116，从站不匹配即拒绝接管。"""
    def __init__(self, adapter):
        self.master = None; self.sdk = None; self.thread = None
        self.running = False; self.last_good = 0.; self.io_error = ''; self.decoded = 0; self.identity_checked = False
        self.output = None; self.output_lock = threading.Lock()
        # LHandProLib 的状态 getter 与 TPDO 解码共享同一个原生对象。旧版让 IO
        # 线程解码的同时由工作线程读取状态，现场出现过 EtherCAT 已 connected、
        # 但第二次 snapshot 不再返回的现象。所有 SDK 调用在本进程内串行化。
        self.sdk_lock = threading.RLock()
        self.initial_snapshot = None
        runtime_paths()
        import pysoem
        from lhandprolib_python_sdk.lhandprolib_wrapper import PyLHandProLib
        self.pysoem = pysoem
        try:
            self.master = pysoem.Master(); self.master.open(adapter)
            if self.master.config_init() != 1:
                raise RuntimeError('要求专用网口上恰好一只 DH116；不自动选择/接管其它设备')
            slave = self.master.slaves[0]
            # 来自用户提供的 LSLQ_DH116.xml，不靠名字猜型号。
            if slave.man != 0x17185 or slave.id != 0x00660000:
                raise RuntimeError(f'ESI 身份不匹配：vendor={slave.man:#x} product={slave.id:#x}')
            self.identity_checked = True
            self.master.config_map(); self.master.config_dc()
            if self.master.state_check(pysoem.SAFEOP_STATE, 100000) != pysoem.SAFEOP_STATE:
                raise RuntimeError('DH116 未进入 SAFEOP；请检查独占网卡/供电/接线')
            slave.output = bytes(len(slave.output))
            self.master.send_processdata(); self.master.receive_processdata(2000)
            self.master.state = pysoem.OP_STATE; self.master.write_state()
            for _ in range(20):
                self.master.send_processdata(); self.master.receive_processdata(2000)
                if self.master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE: break
            else: raise RuntimeError('DH116 未进入 OP')
            self.sdk = PyLHandProLib(str(THIRD_PARTY/'dh116_sdk/LHandProLib.dll'))
            with self.sdk_lock:
                self.sdk.set_send_rpdo_callback(self._send)
            self.running = True
            self.thread = threading.Thread(target=self._io, daemon=True, name='DH116-PDO')
            self.thread.start()
            with self.sdk_lock:
                self.sdk.initial(0)  # LCN_ECAT，只初始化通信/读取配置。
                if self.sdk.get_dof()[1] != 6 or self.sdk.get_hand_direction() != 0:
                    raise RuntimeError('SDK 反馈不是六驱动右手，拒绝继续')
                self.sdk.set_move_no_home(0)  # 绝不绕过厂家回零检查。
            self.initial_snapshot = self.snapshot()  # 确认反馈，不自动使能/回零/移动。
        except BaseException:
            self.close(); raise

    def _send(self, data):
        if self.master is None or len(data) != len(self.master.slaves[0].output): return False
        with self.output_lock: self.output = bytes(data)
        return True

    def _io(self):
        try:
            while self.running:
                with self.output_lock: output = self.output
                if output is not None: self.master.slaves[0].output = output
                self.master.send_processdata()
                wkc = self.master.receive_processdata(2000)
                if wkc >= self.master.expected_wkc and self.master.expected_wkc > 0:
                    with self.sdk_lock:
                        result = self.sdk.set_tpdo_data_decode(bytes(self.master.slaves[0].input))
                    if result == 0:
                        self.last_good = time.monotonic(); self.decoded += 1
                time.sleep(0.002)
        except BaseException as exc:
            self.io_error = str(exc)

    def snapshot(self):
        if self.io_error or time.monotonic()-self.last_good > FEEDBACK_S:
            raise RuntimeError('DH116 反馈超时/WKC异常：'+self.io_error)
        with self.sdk_lock:
            alarms = [self.sdk.get_now_alarm(i) for i in range(1,7)]
            states = [self.sdk.get_now_status(i) for i in range(1,7)]
            positions = [self.sdk.get_now_position(i) for i in range(1,7)]
            angles = [self.sdk.get_now_angle(i) for i in range(1,7)]
            enabled = all(self.sdk.get_enable(i) for i in range(1,7))
        if any(alarms) or any(s not in (0,1) for s in states):
            raise RuntimeError(f'手报警/限位/回零中：alarms={alarms}, states={states}')
        return {'positions':positions, 'angles_deg':angles, 'motors_enabled':enabled}

    def prepare(self, limits):
        """仅显式点击准备后使能；目标先匹配实测，不能重放 LHandPro 的旧目标。"""
        _, speed, current = validate_limits(*limits)
        positions = self.snapshot()['positions']
        with self.sdk_lock:
            self.sdk.stop_motors(0)
            for i, p in enumerate(positions, 1):
                self.sdk.set_control_mode(i, 0)  # LCM_POSITION，避免沿用上位机的速度/力矩模式。
                self.sdk.set_target_position(i, int(p))
                self.sdk.set_position_velocity(i, int(speed*100))
                self.sdk.set_max_current(i, int(current))
            self.sdk.set_safe_current_enable(True)
            self.sdk.set_enable(0, True)
        deadline = time.monotonic()+0.8
        while time.monotonic()<deadline:
            if self.snapshot()['motors_enabled']: return
            time.sleep(0.02)
        raise RuntimeError('手使能确认超时；没有发送开合目标')

    def move(self, positions):
        with self.sdk_lock:
            for i, position in enumerate(positions, 1): self.sdk.set_target_position(i, round(position))
            self.sdk.move_motors(0)

    def stop(self):
        if self.sdk is not None:
            with self.sdk_lock:
                self.sdk.stop_motors(0)

    def close(self):
        # 保持已夹物体时不能擅自张手/掉使能；只请求停止，不自动释放物体。
        if self.sdk is not None:
            try: self.stop()
            except Exception: pass
            # 让 IO 线程有机会发送停止 RPDO；这不是物理停止确认，现场急停仍独立。
            if self.running: time.sleep(0.05)
        self.running = False
        if self.thread is not None: self.thread.join(1.)
        if self.sdk is not None:
            try:
                with self.sdk_lock: self.sdk.close()
            except Exception: pass
        if self.master is not None:
            try:
                if self.identity_checked:
                    self.master.state = self.pysoem.INIT_STATE; self.master.write_state()
                self.master.close()
            except Exception: pass


class DemoHand:
    """离线替身使用同一个许可/限速状态机，不加载驱动、不打开网卡。"""
    def __init__(self): self.positions = [0.]*6; self.enabled = False
    def snapshot(self):
        return {'positions':list(self.positions), 'angles_deg':[p/10000*d for p,d in zip(self.positions,ACTIVE_DEGREES)],
                'motors_enabled':self.enabled}
    def prepare(self, limits): validate_limits(*limits); self.enabled = True
    def move(self, positions): self.positions = list(positions)
    def stop(self): pass
    def close(self): pass


def _put_latest(channel, data):
    try: channel.put_nowait(data)
    except queue.Full:
        try: channel.get_nowait()
        except queue.Empty: pass
        try: channel.put_nowait(data)
        except queue.Full: pass


def hand_worker(adapter, live, commands, output, heartbeat, stop_event, exit_event):
    """授权与看门狗放在子进程；GUI 退出/暂停后旧 Trigger 不能继续驱动。"""
    device = None; policy = TriggerPolicy(); authorized = False; moving = False
    limits = (30., 10., 150.); status = '已连接，尚未准备手控制'; last_fault = ''; reject_before = 0.
    log_dir = ROOT/'Validation/dh116_sessions'; log_dir.mkdir(parents=True, exist_ok=True)
    log = (log_dir/f'hand_{time.time_ns()}.jsonl').open('a',encoding='utf-8')
    def record(event, **fields):
        log.write(json.dumps({'time_ns':time.time_ns(),'event':event,**fields},ensure_ascii=False)+'\n'); log.flush()
    try:
        device = EthercatHand(adapter) if live else DemoHand()
        record('connected', live=live, adapter=adapter)
        # 构造阶段已完成一次真实反馈校验，立即发布。旧版必须等待下一轮
        # snapshot，原生 getter 一旦阻塞就会造成“网卡已占用、界面仍显示未连接”。
        first_snap = getattr(device, 'initial_snapshot', None) or device.snapshot()
        _put_latest(output, {**first_snap, 'connected':True, 'feedback_valid':True,
                             'simulated':not live, 'authorized':False, 'moving':False,
                             'status':status, 'timestamp':time.monotonic()})
        last = time.monotonic()
        while not exit_event.is_set():
            now = time.monotonic(); dt = now-last; last = now
            try:
                stopped_this_tick = stop_event.is_set()
                if stopped_this_tick:
                    reject_before = time.perf_counter_ns()
                    device.stop(); moving=False; authorized=False; policy.reset()
                    status='已停止手，需重新准备'; stop_event.clear(); record('stop')
                    while True:
                        try: commands.get_nowait()
                        except queue.Empty: break
                try: command = commands.get_nowait()
                except queue.Empty: command = None
                if command is not None and not stopped_this_tick and command[2] > reject_before:
                    if command[0] == 'prepare':
                        limits=validate_limits(*command[1]); device.prepare(limits)
                        authorized=True; policy.reset(); status='手已准备；按住 Grip 后先释放 Trigger'; record('prepared',limits=limits)
                snap = device.snapshot()
                if authorized and not snap['motors_enabled']:
                    raise RuntimeError('手使能未确认/已丢失，请检查后重新准备')
                with heartbeat.get_lock(): sent, permit, trigger = heartbeat[:]
                # 必须在读取共享心跳之后取时刻；循环开头的 now 可能早于刚写入的心跳，
                # 尤其 prepare 正在等待 SDK 回应时。拿旧 now 相减会把新包误判成“未来/过期”。
                fresh = 0 <= time.monotonic()-sent <= HEARTBEAT_S
                allowed = authorized and fresh and bool(permit)
                target = policy.step(permit=allowed, trigger=trigger, positions=snap['positions'], dt=dt,
                                     max_percent=limits[0],speed_percent_s=limits[1])
                if target is not None:
                    device.move(target); moving=True
                    status='Trigger 开合中（松 Grip 停止；不自动张手）'
                elif moving:
                    device.stop(); moving=False
                    status='手已暂停；重新 Grip 后先释放 Trigger'
                if not fresh and authorized:
                    reject_before = time.perf_counter_ns()
                    device.stop(); authorized=False; policy.reset(); status='GUI心跳超时，手授权已撤销'; record('watchdog_stop')
                _put_latest(output, {**snap,'connected':True,'feedback_valid':True,'simulated':not live,
                                     'authorized':authorized,'moving':moving,'status':status,'timestamp':time.monotonic()})
            except Exception as exc:
                reject_before = time.perf_counter_ns()
                authorized=False; moving=False; policy.reset(); status='手故障：'+str(exc)
                try: device.stop()
                except Exception as stop_exc: status+='；停止请求失败：'+str(stop_exc)
                if status != last_fault: record('fault',message=status)
                last_fault = status
                _put_latest(output, {'connected':True,'feedback_valid':False,'simulated':not live,
                                     'authorized':False,'status':status,'timestamp':time.monotonic()})
                # 故障锁存：只靠下一帧“恢复正常”不能自动重新开合。
            time.sleep(0.02)
    except BaseException as exc:
        record('connection_failed',message=str(exc))
        _put_latest(output, {'connected':False,'feedback_valid':False,'authorized':False,'status':str(exc),'timestamp':time.monotonic()})
    finally:
        if device is not None: device.close()
        record('closed'); log.close()


class HandController:
    """GUI 侧非阻塞门面。构造对象不会连接硬件，只有 connect 才创建进程。"""
    def __init__(self, *, live):
        self.live=live; self.process=None; self.ctx=mp.get_context('spawn')
        self.connect_started = 0.0
        self.state={'connected':False,'feedback_valid':False,'authorized':False,'simulated':not live,'status':'未连接 DH116'}

    def connect(self, adapter=''):
        if self.process and self.process.is_alive(): raise ValueError('手通道仍在运行，请先断开')
        self.commands=self.ctx.Queue(4); self.output=self.ctx.Queue(4)
        self.heartbeat=self.ctx.Array('d',[time.monotonic(),0.,0.])
        self.stop_event=self.ctx.Event(); self.exit_event=self.ctx.Event()
        self.state={'connected':False,'feedback_valid':False,'authorized':False,'simulated':not self.live,'status':'正在连接 DH116（不自动使能/回零）'}
        self.connect_started = time.monotonic()
        self.process=self.ctx.Process(target=hand_worker,args=(adapter,self.live,self.commands,self.output,self.heartbeat,self.stop_event,self.exit_event),daemon=True)
        self.process.start()

    def update(self, *, permit, trigger):
        if not self.process: return
        with self.heartbeat.get_lock(): self.heartbeat[:]=[time.monotonic(),float(permit),float(trigger)]

    def snapshot(self):
        if self.process:
            while True:
                try: self.state=self.output.get_nowait()
                except queue.Empty: break
            if not self.process.is_alive():
                self.state={**self.state,'connected':False,'feedback_valid':False,'authorized':False}
            elif time.monotonic()-self.state.get('timestamp',0)>0.5:
                self.state={**self.state,'feedback_valid':False,'authorized':False}
                if not self.state.get('connected') and time.monotonic()-self.connect_started > 2.0:
                    self.state={**self.state,
                                'status':'DH116 子进程已打开网卡但尚未返回新鲜反馈；请点“断开手”，不要重复连接'}
        return dict(self.state)

    def prepare(self, limits):
        validate_limits(*limits)
        if not self.snapshot().get('feedback_valid'): raise ValueError('尚无新鲜手反馈')
        # 操作者点击“准备”可能发生在长时间暂停之后、下一次 GUI tick 之前。
        # 先发布“新鲜但禁止运动”的心跳，再排队授权；否则子进程可能立刻用旧心跳撤权。
        self.update(permit=False,trigger=0.)
        # Windows/Python 3.12 的 monotonic 可能是粗粒度 tick；“停止后立即重新准备”
        # 会落在同一 tick。用跨进程 QPC 高精度时间区分新请求与停止前排队的旧请求。
        self.commands.put_nowait(('prepare',limits,time.perf_counter_ns()))

    def stop(self):
        if self.process and self.process.is_alive():
            self.update(permit=False,trigger=0.); self.stop_event.set()

    def close(self):
        if self.process:
            self.stop(); self.exit_event.set(); self.process.join(2.)
            if self.process.is_alive():
                # 不静默宣称“已停机”；原生调用卡住时需要现场物理处理，禁止重新接管。
                raise RuntimeError('DH116 子进程尚未退出，请现场确认物理停止后处理；不可重复连接')
            self.process=None
        self.connect_started = 0.0
        self.state={'connected':False,'feedback_valid':False,'authorized':False,'simulated':not self.live,'status':'DH116 已断开'}
