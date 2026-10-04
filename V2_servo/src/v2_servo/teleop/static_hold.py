"""复用H1–H9有限现场调试循环；每种协议独立前置门和报告，不开放正式全空间。"""
import threading
import time

import numpy as np

from ..configuration import vector
from ..jaka import JakaServo, checked, checked_bool, decode_status
from ..runtime import Deadline
from ..safety import SafetyFault
from .edg import decode_edg_state, decode_status_pair,StaggeredSafetyPoll
from .motion import VelocityServo


class StaticTarget:
    """固定参考不能被改成运动轨迹；阈值是H1实验保护线，不是任务精度验收值。"""
    def __init__(self, q, limits):
        self.q=vector(q)
        if np.any(self.q<limits['lower_rad']) or np.any(self.q>limits['upper_rad']):
            raise SafetyFault('H1 initial joint outside configured limits')
        self.q.setflags(write=False)

    def check(self, candidate, actual, velocity):
        if not np.array_equal(vector(candidate), self.q):
            raise SafetyFault('H1 refuses any change to the measured joint target')
        if np.max(np.abs(vector(actual)-self.q))>np.deg2rad(.2):
            raise SafetyFault('H1 measured joint deviation exceeds 0.2 degree')
        if np.max(np.abs(vector(velocity)))>np.deg2rad(1.):
            raise SafetyFault('H1 measured joint speed exceeds 1 degree/s')


def require_safe(flags):
    if (flags['estop'] or flags['collision'] or flags['soft_limit'] or flags['error']
            or not flags['powered'] or not flags['enabled']):
        raise SafetyFault('H1 raw controller safety fault: '+str(flags))


class StaticStatusWatcher:
    """单SDK所有者内的轻量安全状态线程。未知/超时即失败，不滤波故障位。"""
    def __init__(self, rc, emit):
        self.rc=rc;self.emit=emit;self.stop_event=threading.Event()
        self.lock=threading.Lock();self.flags=None;self.at=0.;self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True,name='H1-raw-safety')

    def run(self):
        poll=StaggeredSafetyPoll(self.rc,self.emit)
        while not self.stop_event.is_set():
            try:
                flags,stamp,wait=poll.query_one()
                if flags is not None:
                    require_safe(flags)
                    with self.lock:self.flags=flags;self.at=stamp;self.error=None
            except Exception as exc:
                with self.lock:self.error=str(exc)
                return
            self.stop_event.wait(wait)

    def get(self):
        with self.lock:flags=self.flags;stamp=self.at;error=self.error
        if error:raise SafetyFault(error)
        if flags is None or time.perf_counter()-stamp>.15:
            raise SafetyFault('H1 lightweight safety feedback missing/stale')
        return flags

    def close(self):
        self.stop_event.set();self.thread.join(1)
        if self.thread.is_alive():raise SafetyFault('H1 status thread blocked; preserve SDK owner')


class StaticFeedback:
    """只衡量新EDG源时间戳抵达后的停滞；不编造绝对采样年龄或时钟同步。"""
    def __init__(self, rc):
        self.rc=rc;self.source=None;self.received=None;self.last_q=None

    def sample(self):
        start=time.perf_counter()
        raw=self.rc.edg_get_stat(0);details=self.rc.edg_stat_details()
        end=time.perf_counter()
        q,dq,tcp,source,count=decode_edg_state(checked(raw,'edg_get_stat'),
                                             checked(details,'edg_stat_details'))
        if self.source is None or source>self.source:
            if self.source is not None and source-self.source>.0241:
                raise SafetyFault('H1 EDG source gap exceeds 24 ms')
            self.source=source;self.received=end
        elif source<self.source:
            raise SafetyFault('H1 EDG source clock reversal')
        if end-self.received>.024:
            raise SafetyFault('H1 EDG source timestamp stopped advancing')
        self.last_q=q
        return q,dq,tcp,{'host_query_start_s':start,'host_return_s':end,
                         'controller_send_timestamp_s':source,'controller_count':count,
                         'absolute_feedback_age_s':None,
                         'state':raw,'details':details}


def preflight(rc, cfg, expected):
    """所有getter均在静态阶段；不自动上电、使能、清报警或修改碰撞设置。"""
    stamp=time.perf_counter()
    if hasattr(rc,'read_safety'):
        raw=rc.read_safety()
        flags=decode_status_pair(checked(raw['motion'],'get_motion_status'),
                                 checked(raw['simple'],'get_robot_status_simple'))
    else:
        raw=rc.get_robot_status();fb=decode_status(raw,stamp,0)
        flags={'estop':fb.estop,'collision':fb.collision,'soft_limit':fb.soft_limit,
               'error':fb.error,'powered':fb.powered,'enabled':fb.enabled}
    require_safe(flags)
    if checked_bool(rc.is_in_servomove(),'is_in_servomove'):
        raise SafetyFault('H1 refuses takeover of existing servo')
    if not checked_bool(rc.is_in_pos(),'is_in_pos'):
        raise SafetyFault('H1 requires stationary robot')
    tool_id=checked(rc.get_tool_id(),'get_tool_id')
    if tool_id!=expected['tool_id']:raise SafetyFault('H1 Tool1 mismatch')
    tool=rc.get_tool_data(tool_id);checked(tool,'get_tool_data')
    if len(tool)!=3 or tool[1]!=tool_id or not np.allclose(vector(tool[2]),expected['tcp_mm_rad'],rtol=0,atol=.001):
        raise SafetyFault('H1 current tool TCP mismatch')
    payload=checked(rc.get_payload(),'get_payload')
    if len(payload)!=2 or abs(float(payload[0])-expected['payload_kg'])>1e-6 or not np.allclose(payload[1],expected['cog_mm'],rtol=0,atol=.001):
        raise SafetyFault('H1 current payload/CoG mismatch')
    level=checked(rc.get_collision_level(),'get_collision_level')
    if type(level) is not int or not 1<=level<=5:
        raise SafetyFault('H1 requires controller collision protection enabled')
    q=vector(checked(rc.get_actual_joint_position(),'get_actual_joint_position'))
    StaticTarget(q,cfg['limits'])
    return {'q_rad':q.tolist(),'status':raw,'tool':tool,'payload':payload,
            'collision_level':level,'tcp':rc.get_actual_tcp_position(),
            'sdk':rc.get_sdk_version()}


def probe_edg0_safety_readonly(rc,cfg,options,expected,stop_event,emit):
    """核查EDG0故障读取；不使能伺服、不调用运动接口。"""
    report={'scope':'live_EDG0_safety_api_readonly_no_motion','servo_commands':0,
            'failure':None,'continuous_teleop_approved':False,'cleanup_errors':[],
            'raw_safety_available':False,'passed_readonly':False,'samples':[]}
    attempted=False
    try:
        if stop_event.is_set():raise SafetyFault('operator stop before EDG0 probe')
        report['preflight']=preflight(rc,cfg,expected)
        attempted=True
        checked(rc.edg_init_extend(True,options['edg_client_ip'],options['edg_port'],0),'EDG0 enable readonly')
        for name in ('get_motion_status','get_robot_status_simple'):
            if stop_event.is_set():raise SafetyFault('operator stop in EDG0 probe')
            start=time.perf_counter();raw=getattr(rc,name)();returned=time.perf_counter()
            row={'api':name,'start_s':start,'returned_s':returned,'raw':raw}
            report['samples'].append(row);emit(event='EDG0_readonly_api',**row)
        motion,simple=(x['raw'] for x in report['samples'])
        require_safe(decode_status_pair(checked(motion,'EDG0 get_motion_status'),
                                        checked(simple,'EDG0 get_robot_status_simple')))
        report['raw_safety_available']=True
        checked(rc.edg_get_stat(0),'EDG0 state');checked(rc.edg_stat_details(),'EDG0 details')
        report['passed_readonly']=True
    except Exception as exc:report['failure']=str(exc)
    finally:
        if attempted:
            try:
                report['edg_exit']=rc.edg_init_extend(False,options['edg_client_ip'],options['edg_port'],0)
                checked(report['edg_exit'],'EDG0 exit')
            except Exception as exc:report['cleanup_errors'].append(str(exc));report['passed_readonly']=False
        try:report['servo_exited']=not checked_bool(rc.is_in_servomove(),'post EDG0 servo state')
        except Exception as exc:report['cleanup_errors'].append(str(exc));report['passed_readonly']=False
    emit(event='EDG0_readonly_result',**report)
    return report


def _run_experiment(rc,cfg,options,expected,stop_event,emit,seconds=.5,protocol='H1',quest=None):
    """受值守监督的限时静态实验；连续遥操作仍须完整部署审查。"""
    if protocol not in ('H1','H2','H3','H4','H5','H6','H7','H8','H9','H10'):raise ValueError('unknown commissioning experiment')
    quest_protocol=protocol in ('H3','H4','H5','H6','H7','H8','H9','H10')
    tcp_protocol=protocol in ('H4','H5','H6','H8','H9','H10')
    tcp_axis=options.get('commissioning_tcp_axis','X') if protocol=='H5' else 'Z'
    rotation_axis=options.get('commissioning_rotation_axis','RX')
    if protocol=='H7' and rotation_axis not in ('RX','RY','RZ'):raise SafetyFault('H7 invalid rotation axis')
    if protocol=='H5' and tcp_axis not in ('X','Y'):raise ValueError('H5 axis must be X/Y')
    if protocol=='H1' and not .5<=seconds<=1.:raise ValueError('H1 duration must be 0.5..1 second')
    if protocol=='H2':seconds=8.
    if quest_protocol:
        seconds=32.
        if quest is None or getattr(rc,'commissioning_edg_mode',1)!=0:raise SafetyFault('H3 requires Quest input and EDG0 candidate')
    if not cfg['filters']['enabled'] or cfg['filters'].get('controller_lpf_policy','preserve')!='preserve':
        raise SafetyFault('H1 requires software filters on and controller LPF preserved')
    result={'scope':{'H1':'live_H1_current_joint_static_hold_only','H2':'live_H2_J1_0p1deg_roundtrip_only','H3':'live_H3_Quest_Grip_stick_J1_0_to_0p1deg_only','H4':'live_H4_Quest_Tool1_base_Z_0_to_0p5mm_only','H5':'live_H5_Quest_Tool1_base_'+tcp_axis+'_0_to_0p5mm_only','H6':'live_H6_Quest_relative_XYZ_translation_local_sphere_only','H7':'live_H7_Quest_Tool1_base_'+rotation_axis+'_local_0p02deg_only','H8':'live_H8_Quest_Tool1_combined_6dof_local_finite_only','H9':'live_H9_Quest_combined_6dof_supervised_expansion_only','H10':'live_H10_Quest_Tool1_nonlinear_workspace_pilot_only'}[protocol],
            'experiment':protocol,'continuous_teleop_approved':False,
            'absolute_feedback_age_validated':False,'network_loss_stop_validated':False,
            'failure':None,'servo_commands':0,'cleanup_errors':[],'servo_exited':None,
            'physical_stationary_observed':False,'samples':[]}
    if quest_protocol:
        result['input_mode']=getattr(quest,'input_mode','joystick')
        if tcp_protocol:result['tcp_axis']='XYZ' if protocol in ('H6','H8','H9','H10') else tcp_axis
        if protocol=='H3' and result['input_mode']=='pose':result['scope']='live_H3_Quest_Grip_relative_hand_Y_J1_0_to_0p1deg_only'
    if protocol=='H7':result['rotation_axis']=rotation_axis
    driver=None;watcher=None;edg_attempted=False;planner=None;started=None
    client=options['edg_client_ip'];port=options['edg_port']
    edg_mode=getattr(rc,'commissioning_edg_mode',1);result['edg_mode']=edg_mode
    result['command_acceptance_scope']=getattr(rc,'command_acceptance_scope','sdk_normal_return')
    deadline=float('inf')
    auto_complete=False;completion_gate=None;rotation_extrema=[float('inf'),-float('inf'),float('inf'),-float('inf')]
    if protocol=='H7' and options.get('commissioning_rotation_auto_complete'):
        from .gesture_completion import GestureCompletion
        completion_gate=GestureCompletion()
    if protocol in ('H8','H9','H10'):
        from .gesture_completion import GestureCompletion
        completion_gate=GestureCompletion()
        pose_min=np.full(12,np.inf);pose_max=-pose_min
    def authorize(action):
        if stop_event.is_set():raise SafetyFault('H1 operator/watchdog stop requested')
        if action=='send' and time.perf_counter()>=deadline:
            raise SafetyFault('H1 fixed duration budget exhausted')
    try:
        authorize('preflight')
        if protocol=='H10':
            from .local_nonlinear import validate_workspace
            validate_workspace(options['quest_workspace_profile'])
            if result['input_mode']!='pose6dof_live':raise SafetyFault('H10 dedicated receiver required')
        if protocol=='H9':
            from .pose_expansion import validate_expansion
            validate_expansion(options['quest_pose_expansion_profile'])
            if result['input_mode']!='pose6dof_live':raise SafetyFault('H9 requires dedicated live receiver')
        if protocol in ('H6','H8'):
            from .quest_tcp_xyz import validate_xyz_profile
            validate_xyz_profile(options['quest_xyz_profile'])
            if result['input_mode']!=('xyz' if protocol=='H6' else 'pose6dof_live'):
                raise SafetyFault('H6 requires dedicated XYZ receiver' if protocol=='H6' else 'H8 requires dedicated combined live receiver')
        if protocol=='H7' and result['input_mode']!='rotation_shadow':raise SafetyFault('H7 requires tested rotation receiver')
        result['preflight']=preflight(rc,cfg,expected)
        emit(event='preflight',**result['preflight'])
        local_model=None
        if tcp_protocol or protocol=='H7':
            from .official_local import local_translation_model
            if checked(rc.get_user_frame_id(),'get_user_frame_id')!=0:
                raise SafetyFault(protocol+' requires current robot base user frame 0; no automatic change')
            if protocol=='H7':
                from .local_rotation import local_rotation_model
                local_model=local_rotation_model(rc,result['preflight']['q_rad'],rotation_axis)
                result['official_local_tcp_rotation']=local_model
                emit(event='H7_local_model',axis=rotation_axis,maximum_angle_deg=local_model['maximum_angle_deg'])
            elif protocol=='H10':
                from .local_nonlinear import workspace_model
                local_model=workspace_model(rc,result['preflight']['q_rad'],options['quest_workspace_profile'],cfg['limits'],emit)
                result['official_local_tcp_xyz']=local_model
            elif protocol=='H9':
                from .pose_expansion import expansion_model
                local_model=expansion_model(rc,result['preflight']['q_rad'],options['quest_pose_expansion_profile'],cfg['limits'])
                result['official_local_tcp_xyz']=local_model
                emit(event='H9_local_model',radius_mm=local_model['maximum_radius_mm'],
                     joint_envelope_deg=local_model['joint_envelope_deg'],profile=options['quest_pose_expansion_profile'])
            elif protocol in ('H6','H8'):
                from .local_xyz import local_xyz_model
                local_model=local_xyz_model(rc,result['preflight']['q_rad'])
                result['official_local_tcp_xyz']=local_model
                emit(event=protocol+'_local_model',radius_mm=local_model['maximum_radius_mm'],gain_rad_per_mm_xyz=local_model['gain_rad_per_mm_xyz'])
            else:
                local_model=local_translation_model(rc,result['preflight']['q_rad'],tcp_axis)
                result['official_local_tcp_Z' if protocol=='H4' else 'official_local_tcp_axis']=local_model
                emit(event=protocol+'_local_model',axis=tcp_axis,maximum_axis_mm=local_model['maximum_axis_mm'],
                     gain_rad_per_mm=local_model['joint_gain_rad_per_mm_'+tcp_axis],
                     weighted_sigma_min=local_model['weighted_singular_values'][-1])
        if protocol in ('H2','H3'):
            # 本次现场审查限于J1微动。官方正解查询只是数学扫掠，不发送运动。
            origin=np.array(result['preflight']['q_rad']);sweep=[]
            for angle in np.linspace(0,np.deg2rad(.1),21):
                candidate=origin.copy();candidate[0]+=angle
                tcp=vector(checked(rc.kine_forward(candidate.tolist()),'H2 kine_forward'))
                sweep.append({'q_rad':candidate.tolist(),'tcp_mm_rad':tcp.tolist()})
            start_tcp=np.array(sweep[0]['tcp_mm_rad'][:3])
            maximum=max(np.linalg.norm(np.array(row['tcp_mm_rad'][:3])-start_tcp) for row in sweep)
            result['official_fk_micro_sweep']={'samples':sweep,'max_tcp_translation_mm':float(maximum),
                                                'collision_clearance_validated':False}
            if maximum>3.:raise SafetyFault('H2 official mathematical TCP sweep unexpectedly exceeds 3 mm')
            emit(event='H2_official_fk_micro_sweep',**result['official_fk_micro_sweep'])
        # 官网1.7.2参数表：mnt=2终止。成功是设置受理，绝不宣称断网保护已验收。
        result['network_setter']=rc.set_network_exception_handle(100,2)
        checked(result['network_setter'],'set_network_exception_handle(100,2)')
        edg_attempted=True
        checked(rc.edg_init_extend(True,client,port,edg_mode),'edg_init_extend(true,mode'+str(edg_mode)+')')
        feedback=StaticFeedback(rc)
        watcher=StaticStatusWatcher(rc,emit);watcher.thread.start()
        quiet=[];warm_until=time.perf_counter()+.3
        initial=StaticTarget(result['preflight']['q_rad'],cfg['limits'])
        while time.perf_counter()<warm_until:
            authorize('warmup')
            q,dq,tcp,raw=feedback.sample();initial.check(initial.q,q,dq)
            quiet.append(q)
            emit(event='edg_stationary_warmup',**raw)
            time.sleep(.008)
        watcher.get()
        if len(quiet)<8 or np.max(np.ptp(quiet,axis=0))>np.deg2rad(.05):
            raise SafetyFault('H1 robot not stable during EDG warmup')
        actual=vector(checked(rc.get_actual_joint_position(),'get_actual_joint_position'))
        q,dq,tcp,raw=feedback.sample()
        if np.max(np.abs(actual-q))>np.deg2rad(.05):
            raise SafetyFault('H1 SDK and EDG joints do not agree within 0.05 degree')
        target=StaticTarget(actual,cfg['limits'])
        if protocol=='H1':planner=VelocityServo(target.q,cfg)
        elif protocol=='H2':
            from .single_axis import J1Roundtrip
            planner=J1Roundtrip(target.q,cfg)
        else:
            if protocol=='H3':
                from .quest_j1 import QuestJ1Jog
                planner=QuestJ1Jog(target.q,cfg)
            else:
                from .quest_tcp_z import QuestTCPZJog
                if np.max(np.abs(target.q-np.array(local_model['q_origin_rad'])))>np.deg2rad(.005):
                    raise SafetyFault(protocol+' robot shifted during static local model preparation')
                origin_tcp=vector(checked(rc.kine_forward(target.q.tolist()),protocol+' official origin'))
                if np.linalg.norm(tcp[:3]-origin_tcp[:3])>.1:
                    raise SafetyFault(protocol+' official Tool1 FK/EDG TCP mismatch')
                if protocol=='H7':
                    from .quest_tcp_rotation import QuestTCPRotationJog
                    planner=QuestTCPRotationJog(target.q,cfg,local_model,origin_tcp)
                    from .official_local import rpy_matrix,rotation_vector
                    if np.linalg.norm(rotation_vector(rpy_matrix(tcp)@rpy_matrix(origin_tcp).T))>np.deg2rad(.01):
                        raise SafetyFault('H7 official Tool1 orientation/EDG mismatch')
                    result['maximum_angle_deg']=planner.maximum
                elif protocol in ('H8','H9','H10'):
                    if protocol=='H10':
                        from .local_nonlinear import QuestWorkspacePlanner
                        planner=QuestWorkspacePlanner(target.q,cfg,local_model,origin_tcp,options['quest_workspace_profile'])
                    elif protocol=='H9':
                        from .pose_expansion import QuestPoseExpansionPlanner
                        planner=QuestPoseExpansionPlanner(target.q,cfg,local_model,origin_tcp,options['quest_pose_expansion_profile'])
                    else:
                        from .quest_pose_live import QuestPoseLivePlanner
                        planner=QuestPoseLivePlanner(target.q,cfg,local_model,origin_tcp,options['quest_xyz_profile'])
                    from .official_local import rpy_matrix,rotation_vector
                    if np.linalg.norm(rotation_vector(rpy_matrix(tcp)@rpy_matrix(origin_tcp).T))>np.deg2rad(.01):
                        raise SafetyFault('H8 official Tool1 orientation/EDG mismatch')
                    result['maximum_radius_mm']=planner.radius;result['maximum_angle_deg']=planner.maximum_rotation_deg
                elif protocol=='H6':
                    from .quest_tcp_xyz import QuestTCPXYZJog
                    planner=QuestTCPXYZJog(target.q,cfg,local_model,origin_tcp,options['quest_xyz_profile'])
                    result['maximum_radius_mm']=planner.radius
                elif protocol=='H4':planner=QuestTCPZJog(target.q,cfg,local_model,origin_tcp)
                else:
                    from .quest_tcp_axis import QuestTCPAxisJog
                    planner=QuestTCPAxisJog(target.q,cfg,local_model,origin_tcp)
                result['tcp_origin_mm_rad']=origin_tcp.tolist()
                if protocol not in ('H6','H7','H8','H9','H10'):result['maximum_z_mm' if protocol=='H4' else 'maximum_axis_mm']=planner.maximum_z_mm
            wait_end=time.perf_counter()+20
            emit(event='H3_wait_release',message='松开右Grip；20秒内需要有效Quest追踪与摇杆字段')
            while not planner.wait_ready(quest.latest,time.perf_counter()):
                authorize('wait_input');watcher.get()
                q,dq,_,_=feedback.sample();target.check(target.q,q,dq)
                if time.perf_counter()>=wait_end:raise SafetyFault('H3 no valid released Quest input within 20 seconds: '+quest.error)
                time.sleep(.008)
            planner.released=True
            if protocol in ('H7','H8','H9','H10'):planner.input_rejected_baseline=quest.rejected
        result['fixed_target_q_rad']=target.q.tolist()
        emit(event='fixed_target',q_rad=target.q.tolist(),filters=cfg['filters'])
        driver=JakaServo(rc,cfg,authorize);driver.start()
        # 模式切换包含同步getter；尚未发送目标，不把启动期间未轮询的间隔
        # 当成运行中丢帧。发送阶段独立建立源时间进展监视，原始时间戳仍原样保存。
        feedback=StaticFeedback(rc)
        emit(event='H1_feedback_epoch_after_servo_confirmation')
        started=time.perf_counter();deadline=started+seconds
        if quest_protocol:
            # 只在厂家确认已进入伺服后发布正式阶段，不能把输入演练当真机跟随。
            emit(event=protocol+'_live_ready',input_mode=result['input_mode'],
                 maximum_axis_mm=planner.maximum_z_mm if tcp_protocol else None,
                 maximum_axis_velocity_mm_s=planner.speed if tcp_protocol else None,
                 maximum_angle_deg=planner.maximum if protocol=='H7' else None,
                 maximum_angular_velocity_deg_s=planner.speed if protocol=='H7' else None,
                 started_s=started,message=('正式Tool1联合6DoF有限跟随；XYZ和姿态同时控制，灵巧手不联动。' if protocol in ('H8','H9','H10') else ('正式Tool1 '+rotation_axis+'有限姿态跟随；XYZ和手部不联动。' if protocol=='H7' else ('正式Tool1三轴相对位置跟随；姿态输入禁用。' if protocol=='H6' else ('正式Tool1基坐标'+tcp_axis+'跟随开始；Grip＋抬手，0..+0.5mm。' if tcp_protocol else '正式真机跟随开始；请再次握右Grip并移动。仅J1。')))))
        scheduler=options.get('commissioning_scheduler',{})
        timer=Deadline(cfg['period_s'],spin_s=scheduler.get('spin_s',.0005))
        emit(event='commissioning_scheduler',spin_s=timer.spin_s,period_s=timer.period,max_lateness_s=cfg['safety']['max_lateness_s'])
        while time.perf_counter()<deadline:
            now,late=timer.wait()
            if now>=deadline:break
            authorize('send')
            if late>cfg['safety']['max_lateness_s']:
                result['deadline_miss']={'wake_s':now,'expected_s':now-late,'lateness_ms':late*1000,
                                         'limit_ms':cfg['safety']['max_lateness_s']*1000,'last_phase':planner.phase if protocol!='H1' else 'hold'}
                emit(event='deadline_miss',**result['deadline_miss'])
                raise SafetyFault(protocol+f' scheduler deadline miss: {late*1000:.3f} ms > {cfg["safety"]["max_lateness_s"]*1000:.3f} ms')
            watcher.get()
            safety_returned=time.perf_counter()
            q,dq,tcp,raw=feedback.sample()
            feedback_returned=time.perf_counter()
            if protocol=='H1':motion=planner.step([0.]*6,hold=True)
            elif protocol=='H2':motion=planner.step()
            else:
                if protocol in ('H7','H8','H9','H10') and quest.rejected>planner.input_rejected_baseline:
                    result['input_rejection']={'rejected_count':quest.rejected,'warmup_rejected':planner.input_rejected_baseline,'reason':quest.error}
                    raise SafetyFault(protocol+' active Quest packet rejected; protective stop: '+quest.error)
                intent=quest.latest;input_now=time.perf_counter()
                if protocol in ('H9','H10'):planner.observe_feedback(q,dq)
                if protocol=='H10':planner.check_model_feedback(q,tcp)
                motion=planner.step(intent,input_now,finish=auto_complete or input_now-started>=30.,software_stop=quest.take_software_stop())
            planner_returned=time.perf_counter()
            if protocol=='H1':target.check(motion.q,q,dq)
            else:planner.check(motion.q,q,dq)
            if tcp_protocol or protocol=='H7':planner.check_tcp(tcp)
            call_start=time.perf_counter()
            send_lateness=late+call_start-now
            if send_lateness>cfg['safety']['max_lateness_s']:
                result['deadline_miss']={'wake_s':now,'send_attempt_s':call_start,'expected_s':now-late,
                                         'lateness_ms':send_lateness*1000,'preparation_ms':(call_start-now)*1000,
                                         'limit_ms':cfg['safety']['max_lateness_s']*1000,
                                         'stage_ms':{'raw_safety':(safety_returned-now)*1000,
                                                     'feedback':(feedback_returned-safety_returned)*1000,
                                                     'planning':(planner_returned-feedback_returned)*1000,
                                                     'limit_check':(call_start-planner_returned)*1000},
                                         'last_phase':planner.phase if protocol!='H1' else 'hold'}
                emit(event='pre_send_deadline_miss',**result['deadline_miss'])
                raise SafetyFault(protocol+f' pre-send deadline miss: {send_lateness*1000:.3f} ms > {cfg["safety"]["max_lateness_s"]*1000:.3f} ms')
            emit(event=protocol+'_servo_attempt',send_started_s=call_start,q_target=motion.q.tolist())
            send_error=None
            try:driver.send(motion)
            except BaseException as exc:send_error=exc
            record=driver.last_send or {}
            row={'send_started_s':call_start,'call_ms':None if record.get('elapsed_s') is None else record['elapsed_s']*1000,
                 'native_send':record,'send_error':None if send_error is None else str(send_error),'lateness_ms':late*1000,
                 'send_lateness_ms':send_lateness*1000,'preparation_ms':(call_start-now)*1000,
                 'stage_ms':{'raw_safety':(safety_returned-now)*1000,
                             'feedback':(feedback_returned-safety_returned)*1000,
                             'planning':(planner_returned-feedback_returned)*1000,
                             'limit_check':(call_start-planner_returned)*1000},
                 'q_target':motion.q.tolist(),'q_actual':q.tolist(),'dq_raw':dq.tolist(),
                 'tcp_raw':tcp.tolist(),'v_command':motion.v.tolist(),'a_command':motion.a.tolist(),
                 'j_command':motion.j.tolist(),'phase':'hold' if protocol=='H1' else planner.phase,**raw}
            if quest_protocol:
                row.update(quest_intent=planner.last_intent,input_error=quest.error,
                           input_age_ms=None if intent is None else (input_now-intent['stamp'])*1000,
                           J1_actual_delta_deg=float(np.rad2deg(q[0]-target.q[0])),stop_reason=planner.stop_reason)
                if protocol=='H7' and intent is not None:
                    row['rotation_input_audit']={k:intent.get(k) for k in ('rotation_intent_raw_deg','rotation_input_saturated','rotation_mapping','hand_input_cap_deg')}
                if protocol=='H4':row.update(TCP_Z_actual_delta_mm=float(tcp[2]-planner.origin_tcp[2]),
                                             TCP_Z_target_delta_mm=planner.z_target_mm)
                if protocol=='H5':row.update(tcp_axis=tcp_axis,
                    TCP_axis_actual_delta_mm=float(tcp['XYZ'.index(tcp_axis)]-planner.origin_tcp['XYZ'.index(tcp_axis)]),
                    TCP_axis_target_delta_mm=planner.axis_target_mm)
                if protocol=='H6':row.update(TCP_xyz_actual_delta_mm=(tcp[:3]-planner.origin_tcp[:3]).tolist(),
                    TCP_xyz_command_estimate_mm=planner.xyz_target_mm.tolist(),TCP_xyz_goal_mm=planner.goal_mm.tolist(),
                    brake_bounds=planner.last_brake_bounds,boundary_brakes=planner.boundary_brakes)
                if protocol=='H7':row.update(TCP_rotation_actual_deg=planner.raw_rotation_deg.tolist(),
                    TCP_rotation_command_deg=planner.angle,TCP_translation_actual_delta_mm=(tcp[:3]-planner.origin_tcp[:3]).tolist())
                if protocol in ('H8','H9','H10'):row.update(TCP_xyz_actual_delta_mm=(tcp[:3]-planner.origin_tcp[:3]).tolist(),
                    TCP_xyz_command_estimate_mm=planner.xyz_target_mm.tolist(),TCP_xyz_goal_mm=planner.goal_mm.tolist(),
                    TCP_rotation_actual_deg=planner.raw_rotation_deg.tolist(),TCP_rotation_command_deg=planner.rotation_target_deg.tolist(),
                    brake_bounds=planner.last_brake_bounds,boundary_brakes=planner.boundary_brakes)
                if protocol in ('H9','H10'):row['tracking_governor']=planner.tracking_audit.copy()
            result['samples'].append(row);result['servo_commands']=driver.sent
            emit(event=protocol+'_servo_sample',**row)
            if send_error is not None:raise send_error
            if protocol=='H7' and completion_gate is not None and not auto_complete:
                c=row['TCP_rotation_command_deg'];a=row['TCP_rotation_actual_deg'][('RX','RY','RZ').index(rotation_axis)]
                rotation_extrema=[min(rotation_extrema[0],c),max(rotation_extrema[1],c),min(rotation_extrema[2],a),max(rotation_extrema[3],a)]
                command_span=rotation_extrema[1]-rotation_extrema[0];actual_span=rotation_extrema[3]-rotation_extrema[2]
                if completion_gate.observe(input_now,intent,planner,q,dq,command_span,actual_span):
                    auto_complete=True;result['completion_reason']='motion_two_moving_releases_and_stable_released_feedback'
                    emit(event='H7_gesture_complete',axis=rotation_axis,command_span_deg=command_span,actual_span_deg=actual_span,
                         message='动作及释放已完成，静止门通过；正常减速退出，不必等待30秒。')
            if protocol in ('H8','H9','H10') and not auto_complete:
                values=np.r_[planner.xyz_target_mm,planner.rotation_target_deg,row['TCP_xyz_actual_delta_mm'],planner.raw_rotation_deg]
                pose_min=np.minimum(pose_min,values);pose_max=np.maximum(pose_max,values)
                threshold=[.2,.2,.2,.01,.01,.01] if protocol in ('H9','H10') else [.02,.02,.02,.003,.003,.003]
                covered=bool(np.all(pose_max-pose_min>=np.tile(threshold,2)))
                if protocol=='H10':
                    span=pose_max-pose_min
                    covered=bool(min(np.linalg.norm(span[:3]),np.linalg.norm(span[6:9]))>=1. and min(np.linalg.norm(span[3:6]),np.linalg.norm(span[9:]))>=.05)
                if completion_gate.observe(input_now,intent,planner,q,dq,.006 if covered else 0.,.006 if covered else 0.):
                    auto_complete=True;result['completion_reason']='combined_six_command_and_feedback_spans_two_moving_releases_stable'
                    emit(event=protocol+'_gesture_complete',spans=(pose_max-pose_min).tolist())
            if protocol in ('H2','H3','H4','H5','H6','H7','H8','H9','H10') and planner.finished:break
        result['hold_elapsed_s']=time.perf_counter()-started
        if protocol=='H2' and not planner.finished:raise SafetyFault('H2 trajectory did not finish before 8 second limit')
        if quest_protocol:
            result['release_stops']=planner.release_stops;result['input_stop_reason']=planner.stop_reason
            if not planner.finished:raise SafetyFault('H3 controlled stop exceeded session budget')
    except BaseException as exc:
        result['failure']=str(exc)
    finally:
        if started is not None:result['hold_elapsed_s']=time.perf_counter()-started
        # 先尝试控制器停止，再等线程。故障停止不能为日志/线程退出而延迟。
        if driver is not None:
            result['servo_commands']=driver.sent
            begin=time.perf_counter()
            try:
                result['stop']=driver.stop();result['servo_exited']=result['stop'].get('servo_exited')
                result['cleanup_errors'].extend(result['stop'].get('errors',[]))
            except Exception as exc:result['cleanup_errors'].append(str(exc))
            result['stop_api_elapsed_s']=time.perf_counter()-begin
        # 故障也保留已发生的Grip释放和运行时长；不能因异常跳过报告字段。
        if quest_protocol and planner is not None:
            result['release_stops']=planner.release_stops
            result['input_stop_reason']=planner.stop_reason
            if protocol in ('H9','H10'):result['tracking_governor_final']=dict(planner.tracking_audit,
                slow_count=planner.tracking_slow_count,reversal_wait_count=planner.tracking_reversal_wait_count)
            if protocol in ('H9','H10') and result['failure']:result['tracking_check_context']=planner.last_tracking_check
        if watcher is not None:
            try:watcher.close()
            except Exception as exc:result['cleanup_errors'].append(str(exc))
        if edg_attempted and (watcher is None or not watcher.thread.is_alive()):
            try:
                result['edg_exit']=rc.edg_init_extend(False,client,port,edg_mode)
                checked(result['edg_exit'],'edg_init_extend(false)')
            except Exception as exc:result['cleanup_errors'].append(str(exc))
        if driver is not None and result['servo_exited'] is True and not result['cleanup_errors']:
            try:
                observed=[]
                for _ in range(8):
                    observed.append(vector(checked(rc.get_actual_joint_position(),'get_actual_joint_position')))
                    time.sleep(.025)
                result['post_stop_q_rad']=[q.tolist() for q in observed]
                result['physical_stationary_observed']=bool(np.max(np.ptp(observed,axis=0))<np.deg2rad(.05))
            except Exception as exc:result['cleanup_errors'].append(str(exc))
        passed=bool(not result['failure'] and not result['cleanup_errors']
                                         and result['servo_commands']>=30
                                         and result['servo_exited'] is True
                                         and result['physical_stationary_observed'])
        result['passed_static_only']=passed and protocol=='H1'
        if protocol=='H2':
            rows=result['samples'];q0=np.array(result.get('fixed_target_q_rad',[0.]*6))
            maximum=0. if not rows else max(r['q_actual'][0]-q0[0] for r in rows)
            end_q=result.get('post_stop_q_rad')
            final_error=None if not end_q else float(np.max(np.abs(np.array(end_q[-1])-q0)))
            result['maximum_measured_J1_delta_deg']=float(np.rad2deg(maximum))
            result['final_joint_error_deg']=None if final_error is None else float(np.rad2deg(final_error))
            result['passed_single_axis_only']=bool(passed and maximum>=np.deg2rad(.07)
                                                   and final_error is not None and final_error<np.deg2rad(.01))
        if protocol=='H3':
            rows=result['samples'];origin=result.get('fixed_target_q_rad',[0]*6)
            span=0. if not rows else max(abs(row['q_actual'][0]-origin[0]) for row in rows)
            command_span=0. if not rows else max(abs(row['q_target'][0]-origin[0]) for row in rows)
            result['grip_active_samples']=sum(bool((row.get('quest_intent') or {}).get('deadman')) for row in rows)
            result['nonzero_intent_samples']=sum(abs((row.get('quest_intent') or {}).get('axis_y') or
                                                     (row.get('quest_intent') or {}).get('stick_y') or 0)>.15 for row in rows)
            result['command_J1_span_deg']=float(np.rad2deg(command_span))
            result['measured_J1_span_deg']=float(np.rad2deg(span))
            result['passed_quest_j1_session_only']=bool(passed and result.get('release_stops',0)>0
                and result['grip_active_samples']>0 and result['nonzero_intent_samples']>0
                and command_span>=np.deg2rad(.005) and span>=np.deg2rad(.005))
            result['quest_6dof_approved']=False
        if protocol in ('H4','H5'):
            rows=result['samples'];origin=result.get('tcp_origin_mm_rad',[0.]*6)
            index='XYZ'.index(tcp_axis)
            actual_span=max((abs(row['tcp_raw'][index]-origin[index]) for row in rows),default=0.)
            command_span=max((abs(row['TCP_Z_target_delta_mm' if protocol=='H4' else 'TCP_axis_target_delta_mm']) for row in rows),default=0.)
            result['measured_TCP_'+tcp_axis+'_span_mm']=actual_span;result['command_TCP_'+tcp_axis+'_span_mm']=command_span
            result['passed_quest_tcp_z_session_only' if protocol=='H4' else 'passed_quest_tcp_axis_session_only']=bool(passed and result.get('release_stops',0)>0
                and actual_span>=.02 and command_span>=.02)
            result['quest_6dof_approved']=False
        if protocol=='H6':
            rows=result['samples']
            actual=np.array([r['TCP_xyz_actual_delta_mm'] for r in rows]) if rows else np.zeros((1,3))
            command=np.array([r['TCP_xyz_command_estimate_mm'] for r in rows]) if rows else np.zeros((1,3))
            result['actual_XYZ_spans_mm']=np.ptp(actual,axis=0).tolist()
            result['command_XYZ_spans_mm']=np.ptp(command,axis=0).tolist()
            result['boundary_brakes']=0 if planner is None else planner.boundary_brakes
            result['release_events']=[] if planner is None else planner.release_events
            result['moving_release_count']=sum(e['moving_at_release'] for e in result['release_events'])
            if planner is not None and planner.stop_reason in ('Quest input lost/stale','Quest tracking invalid'):
                result['failure']=result['failure'] or ('H6 '+planner.stop_reason+'; controlled stop completed, session not accepted')
            result['passed_quest_tcp_xyz_session_only']=bool(passed and result.get('release_stops',0)>0
                and not result['failure'] and result['moving_release_count']>0 and np.all(np.ptp(actual,axis=0)>=.02) and np.all(np.ptp(command,axis=0)>=.02))
            result['quest_6dof_approved']=False
        if protocol=='H7':
            rows=result['samples'];result['release_events']=[] if planner is None else planner.release_events
            result['command_rotation_span_deg']=float(np.ptp([r['TCP_rotation_command_deg'] for r in rows])) if rows else 0.
            result['actual_rotation_span_deg']=float(np.ptp([r['TCP_rotation_actual_deg'][('RX','RY','RZ').index(rotation_axis)] for r in rows])) if rows else 0.
            result['command_'+rotation_axis+'_span_deg']=result['command_rotation_span_deg']
            result['actual_'+rotation_axis+'_span_deg']=result['actual_rotation_span_deg']
            result['moving_release_count']=sum(bool(e['moving_at_release']) for e in result['release_events'])
            if planner is not None and planner.stop_reason in ('Quest input lost/stale','Quest tracking invalid'):
                result['failure']=result['failure'] or ('H7 '+planner.stop_reason+'; session not accepted')
            result['passed_quest_tcp_rotation_session_only']=bool(passed and not result['failure'] and
                result['command_rotation_span_deg']>=.005 and result['actual_rotation_span_deg']>=.005 and result['moving_release_count']>0)
            result['physical_release_stop_validated']=False
        if protocol in ('H8','H9','H10'):
            rows=result['samples'];result['release_events']=[] if planner is None else planner.release_events
            result['moving_release_count']=sum(bool(e['moving_at_release']) for e in result['release_events'])
            for label,key in (('command_XYZ_spans_mm','TCP_xyz_command_estimate_mm'),('actual_XYZ_spans_mm','TCP_xyz_actual_delta_mm'),
                              ('command_rotation_spans_deg','TCP_rotation_command_deg'),('actual_rotation_spans_deg','TCP_rotation_actual_deg')):
                result[label]=np.ptp([r[key] for r in rows],axis=0).tolist() if rows else [0.]*3
            if planner is not None and planner.stop_reason not in (None,protocol+' duration stop'):
                result['failure']=result['failure'] or (protocol+' '+str(planner.stop_reason)+'; session not accepted')
            thresholds=(.2,.01) if protocol in ('H9','H10') else (.02,.003)
            pose_passed=bool(passed and not result['failure'] and result['moving_release_count']>=2
                and all(np.all(np.array(result[k])>=threshold) for k,threshold in
                        (('command_XYZ_spans_mm',thresholds[0]),('actual_XYZ_spans_mm',thresholds[0]),('command_rotation_spans_deg',thresholds[1]),('actual_rotation_spans_deg',thresholds[1]))))
            result['passed_quest_pose_finite_session_only']=pose_passed if protocol=='H8' else False
            if protocol=='H9':
                result['passed_quest_expansion_finite_only']=pose_passed
                result['coverage_missing']=[label+' '+axis for label,axes,threshold in
                    (('command_XYZ_spans_mm',('X','Y','Z'),.2),('actual_XYZ_spans_mm',('X','Y','Z'),.2),
                     ('command_rotation_spans_deg',('RX','RY','RZ'),.01),('actual_rotation_spans_deg',('RX','RY','RZ'),.01))
                    for axis,value in zip(axes,result[label]) if value<threshold]
            result['stable_session_observed']=bool(passed and not result['failure'])
            if protocol=='H10':
                result['passed_workspace_pilot_only']=bool(passed and not result['failure'] and result['moving_release_count']>=2 and
                    min(np.linalg.norm(result['command_XYZ_spans_mm']),np.linalg.norm(result['actual_XYZ_spans_mm']))>=1. and
                    min(np.linalg.norm(result['command_rotation_spans_deg']),np.linalg.norm(result['actual_rotation_spans_deg']))>=.05)
                result['all_six_axes_coverage_validated']=pose_passed
            result['physical_release_stop_validated']=False;result['quest_6dof_approved']=False
        emit(event=protocol+'_result',**{k:v for k,v in result.items() if k!='samples'})
    return result


def run_static_hold(rc,cfg,options,expected,stop_event,emit,seconds=.5):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,seconds,'H1')


def run_single_axis_roundtrip(rc,cfg,options,expected,stop_event,emit):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol='H2')


def run_quest_j1(rc,cfg,options,expected,stop_event,emit,quest):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol='H3',quest=quest)


def run_quest_tcp_z(rc,cfg,options,expected,stop_event,emit,quest):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol='H4',quest=quest)


def run_quest_tcp_axis(rc,cfg,options,expected,stop_event,emit,quest):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol='H5',quest=quest)


def run_quest_tcp_xyz(rc,cfg,options,expected,stop_event,emit,quest):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol='H6',quest=quest)


def run_quest_tcp_rotation(rc,cfg,options,expected,stop_event,emit,quest):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol='H7',quest=quest)


def run_quest_pose_finite(rc,cfg,options,expected,stop_event,emit,quest):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol='H8',quest=quest)


def run_quest_pose_expansion(rc,cfg,options,expected,stop_event,emit,quest):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol='H9',quest=quest)


def run_quest_workspace_pilot(rc,cfg,options,expected,stop_event,emit,quest):
    return _run_experiment(rc,cfg,options,expected,stop_event,emit,protocol="H10",quest=quest)
