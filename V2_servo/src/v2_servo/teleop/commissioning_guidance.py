"""把现场阶段写入既有孪生的显示文件；不连接SDK，不授予运动权限。

仅由预检/监督进程调用。临时文件+原子替换保留最新提示，8ms子进程不碰磁盘。
"""
import json,os,time,uuid
from pathlib import Path
from ..configuration import ROOT


GUIDANCE_FILE=ROOT/'digital_twin/data/quest_live_guidance.json'
STAGES={
    'POSE_WAIT':('联合6DoF等待握持 · 不控制机器人','这是RZ之后的新阶段。先完全松Grip、保持追踪；出现等待首次握持后，重新握Grip再斜向移动并小转。计时从首次握持开始，机器人命令0。'),
    'POSE_SHADOW':('联合6DoF影子 · 不控制机器人','握Grip后斜向移动手柄并小幅转动；运动中松Grip，停稳后反向重复。XYZ与姿态候选同时计算，机器人运动命令0。'),
    'INPUT_READY_WAIT':('等待手柄就绪 · 机器人尚未连接','戴好头显，保持Quest Link与右手柄追踪。松右Grip、摇杆回中；输入稳定后自动开始静态保持，再等绿色正式跟随。'),
    'INPUT_RELEASE':('输入演练 · 不动机械臂','先松开右Grip，摇杆回中。'),
    'INPUT_GESTURE':('输入演练 · 不动机械臂','pose'),
    'INPUT_X':('三方向演练 · 不动机械臂','握右Grip，向前移动手柄约2cm，观察映射+X；另外两轴保持接近0，然后松Grip。'),
    'INPUT_Y':('三方向演练 · 不动机械臂','握右Grip，向左移动手柄约2cm，观察映射+Y；另外两轴保持接近0，然后松Grip。'),
    'INPUT_Z':('三方向演练 · 不动机械臂','握右Grip，上抬手柄约2cm，观察映射+Z；另外两轴保持接近0，然后松Grip。'),
    'ROTATION_SHADOW':('姿态影子演练 · 不动机械臂','先松Grip；握右Grip后分别绕三个方向小幅转动约3–5度并松手。观察RX/RY/RZ读回；影子关节目标不会发送机器人。'),
    'ROTATION_RX':('1/3 · RX旋转读回 · 不动机械臂','握右Grip，小幅转动3–5度；看RX超过1度。转幅不得超过30度。'),
    'ROTATION_RY':('2/3 · RY旋转读回 · 不动机械臂','重新握右Grip，小幅转动3–5度；看RY超过1度。'),
    'ROTATION_RZ':('3/3 · RZ旋转读回 · 不动机械臂','重新握右Grip，小幅转动3–5度；看RZ超过1度。'),
    'ROTATION_RELEASE':('本轴读回完成 · 现在完全松Grip','完全松开右Grip，保持摇杆回中。等待候选轨迹停稳后自动切换下一轴。'),
    'INPUT_RELEASE_FINAL':('演练最后一步 · 不动机械臂','松开右Grip。演练结束后还要在正式阶段再做一次动作。'),
    'INPUT_PASSED':('输入演练已通过','接下来是自动H1/H2；保持Grip松开，等待绿色正式跟随提示。'),
    'H1':('H1 · 真机静态保持','保持手柄松开；机械臂保持当前关节位置。'),
    'H2':('H2 · 真机自动往返','保持手柄松开；仅J1自动+0.1度再返回。这还不是手柄跟随。'),
    'PREPARING':('准备正式真机跟随','保持Grip松开。等绿色提示和两声提示音，再做握持动作。'),
    'LIVE':('正式真机跟随 · 仅J1','pose'),
    'STOPPING':('软件锁停 · 减速退出','松开Grip；等待退出伺服。需要重启本入口才能再次跟随。'),
    'ENDED':('测试已结束','本次结果请看控制台/报告。当前不接受新的运动输入。'),
    'ERROR':('试验未通过 / 已请求停止','松开Grip，查看控制台错误。现场异常立即用物理急停。'),
}


def guidance_packet(stage,input_mode='joystick',detail='',session='',control_scope='J1'):
    if stage not in STAGES:raise ValueError('unknown display stage: '+str(stage))
    if input_mode not in ('pose','joystick','xyz','rotation_shadow','pose6dof_shadow','pose6dof_live'):raise ValueError('unknown display input mode')
    if control_scope not in ('J1','TCP_Z','TCP_X','TCP_Y','TCP_XYZ','TCP_RX','TCP_RY','TCP_RZ','TCP_6DOF','TCP_6DOF_EXPANDED','TCP_6DOF_WORKSPACE','TCP_6DOF_COVERAGE','ROTATION_SHADOW','POSE_SHADOW','CONTINUOUS_SHADOW'):raise ValueError('unknown commissioning control scope')
    title,instruction=STAGES[stage]
    if instruction=='pose':
        if stage=='INPUT_GESTURE':
            instruction=('按住右Grip，再上抬手柄约2cm，随后松Grip。'
                         if input_mode=='pose' else '按住右Grip，将右摇杆推上，随后松Grip。')
            instruction+='这里只检查输入，不驱动机械臂。'
        else:
            instruction=('按住右Grip，再上抬手柄约2cm并保持2秒；松Grip减速。'
                         if input_mode=='pose' else '按住右Grip，再将右摇杆推上并保持2秒；松Grip减速。')
            instruction+=' 完成移动并松手后，再点击摇杆结束。'
    if control_scope=='TCP_6DOF_WORKSPACE' and stage=='LIVE':
        title='正式真机跟随 · Tool1局部非线性10mm候选测试'
        instruction='Grip＋自然缓移/转腕；松Grip减速，停稳可重握。不要求逐轴打卡。手柄不是1:1映射：平移×0.5，转腕×0.02；位置目标≤核验半径75%，姿态目标≤0.3度（保护边界0.5度）。最多30秒，手部禁用；摇杆点击=软件停止。'
    elif control_scope=='TCP_6DOF_COVERAGE' and stage=='LIVE':
        title='正式真机跟随 · H9仅补缺失覆盖'
        instruction='只补下方缺项。当前缺X：Grip＋缓移手柄，使输入X持续增大；保持5–8秒，运动中松Grip。停稳再反向重复。其余轴已有真实证据，不需重做；最多30秒，手部禁用。'
    elif control_scope=='TCP_6DOF_EXPANDED' and stage=='LIVE':
        title='正式真机跟随 · Tool1联合6DoF扩展有限测试'
        instruction='现在重新握Grip，缓慢斜移约2cm并小转3–5度；运动中松Grip，停稳后反向重复。XYZ和姿态同时跟随；半径由当前官方模型核验，至多5mm、姿态至多0.2度、最多30秒；手部禁用。不是全空间放行。'
    elif control_scope=='TCP_6DOF' and stage=='LIVE':
        title='正式真机跟随 · Tool1联合6DoF有限测试'
        instruction='现在重新握Grip，斜向移动约2cm并小转3–5度；运动中松Grip，停稳后反向重复。XYZ和姿态同时跟随；局部半径≤0.5mm、姿态≤0.02度，最多30秒；手部不联动。'
    elif control_scope in ('TCP_RX','TCP_RY','TCP_RZ') and stage=='LIVE':
        title='正式真机跟随 · Tool1 '+control_scope[4:]+'有限姿态'
        instruction='握右Grip后小转3–5度；运动中松Grip。停稳重握后反向小转并松Grip；完成两次运动中释放且静止门通过后自动结束，最多30秒。仅'+control_scope[4:]+'≤±0.02度，XYZ/灵巧手不联动。'
    elif control_scope=='TCP_XYZ' and stage=='LIVE':
        title='正式真机跟随 · Tool1三轴平移'
        instruction='握右Grip后缓慢向前/左/上小幅移动手柄；移动中松Grip。重新握持前等待停稳。最后做一次小斜向移动，松手等30秒退出。姿态输入禁用。'
    elif control_scope.startswith('TCP_') and stage=='LIVE':
        title='正式真机跟随 · Tool1基坐标'+control_scope[-1]
        instruction='按住右Grip后上抬手柄约2cm，保持1秒再松Grip。当前只控制'+control_scope[-1]+'轴，上限不超过+0.5mm；松手后等30秒自动退出，异常随时停止。'
    if control_scope=='TCP_XYZ' and stage=='INPUT_PASSED':instruction='三方向演练已通过；接下来仅复核H1，保持Grip松开，等绿色Tool1三轴平移。'
    if control_scope=='ROTATION_SHADOW' and stage=='LIVE':raise ValueError('rotation shadow cannot show LIVE motion permission')
    if control_scope=='POSE_SHADOW' and stage=='LIVE':raise ValueError('6DoF shadow cannot show LIVE motion permission')
    if input_mode=='pose6dof_shadow' and stage=='LIVE':raise ValueError('6DoF shadow cannot show LIVE motion permission')
    if input_mode=='pose6dof_live' and control_scope not in ('TCP_6DOF','TCP_6DOF_EXPANDED','TCP_6DOF_WORKSPACE','TCP_6DOF_COVERAGE'):raise ValueError('live pose input requires explicit H8/H9/H10 scope')
    if input_mode=='rotation_shadow' and stage=='LIVE' and control_scope not in ('TCP_RX','TCP_RY','TCP_RZ'):raise ValueError('tested rotation input requires explicit RX live scope')
    packet={'schema':1,'scope':'operator_display_only_no_motion_permission','stage':stage,
            'title':title,'instruction':instruction,'detail':detail,'input_mode':input_mode,
            'session':session,'sent_wall_unix_s':time.time(),
            'control_scope':('Tool1联合6DoF · 有限局部测试 · 手部禁用' if control_scope=='TCP_6DOF' else ('6DoF影子 · 机器人命令0 · 候选不是实测反馈' if control_scope=='POSE_SHADOW' else ('Tool1基坐标'+control_scope[4:]+'≤±0.02度 · 仅有限姿态试验' if control_scope in ('TCP_RX','TCP_RY','TCP_RZ') else ('姿态影子 · 机器人运动命令0 · RX候选不下发' if control_scope=='ROTATION_SHADOW' else ('Tool1三轴平移 · 局部半径≤0.5mm · 姿态输入禁用' if control_scope=='TCP_XYZ' else ('Tool1基坐标'+control_scope[-1]+' 0..+0.5mm · 六关节协同' if control_scope.startswith('TCP_') else '仅J1 0..+0.1°'))))))}
    if control_scope=='TCP_6DOF_EXPANDED':packet['control_scope']='Tool1联合6DoF扩展 · 当前模型半径≤5mm · 手部禁用 · 非全空间放行'
    if control_scope=='TCP_6DOF_WORKSPACE':packet['control_scope']='Tool1局部非线性 · 当前半径≤10mm · 姿态≤0.5度 · 非全空间放行'
    if control_scope=='TCP_6DOF_COVERAGE':packet['control_scope']='原H9保护/滤波/限幅不变 · 仅补缺项 · 实测覆盖才接H10'
    if control_scope=='CONTINUOUS_SHADOW':
        if stage=='LIVE':raise ValueError('continuous shadow cannot grant live motion')
        packet['title']='连续大范围影子运行 · 不控制真机'
        packet['instruction']='握Grip自然移动/转腕；松开停稳后可重握。没有起点10mm小球或固定30秒限制。孪生显示模拟反馈；摇杆点击停止。\n'+detail
        packet['detail']=''
        packet['control_scope']='候选规划与模拟反馈 · 机器人运动命令0 · 非真机验收'
    return packet


def workspace_mapping_detail(row):
    """仅在监督进程格式化，计划指令和机器人反馈分开标注。"""
    import math
    def vec(key,digits):return str([round(float(x),digits) for x in row[key]])
    def norm(key):return math.sqrt(sum(float(x)**2 for x in row[key]))
    return ('手柄位移mm='+vec('hand_base_displacement_mm',1)+
        ' → 限幅目标mm='+vec('robot_goal_mm',2)+'\n'+
        '规划指令mm='+vec('robot_command_mm',2)+' | 真机反馈mm='+vec('TCP_xyz_actual_delta_mm',2)+'\n'+
        '手柄转幅='+format(norm('hand_rotation_deg'),'.1f')+'° → 规划指令='+
        format(norm('robot_target_rotation_deg'),'.3f')+'° | 真机反馈='+
        format(norm('TCP_rotation_actual_deg'),'.3f')+'°；姿态目标封顶0.300°；'+row['phase'])


class GuidanceWriter:
    def __init__(self,path=GUIDANCE_FILE):
        self.path=Path(path);self.last_write=-float('inf')
        self.last_stage=None;self.errors=0
    def update(self,stage,input_mode='joystick',detail='',session='',force=False,control_scope='J1'):
        # 显示故障不得产生SDK指令，也不得让监督进程失去保护/清理机会。
        now=time.perf_counter()
        if not force and stage==self.last_stage and now-self.last_write<.2:return False
        temporary=self.path.with_name(self.path.name+'.'+uuid.uuid4().hex+'.tmp')
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            temporary.write_text(json.dumps(guidance_packet(stage,input_mode,detail,session,control_scope),
                                            ensure_ascii=False,allow_nan=False),encoding='utf-8')
            os.replace(temporary,self.path)
            self.last_write=now;self.last_stage=stage
            return True
        except OSError:
            self.errors+=1
            return False
        finally:
            if temporary.exists():temporary.unlink()
