"""厂家接口边界：位置-力矩混合模式存在；不虚构牛顿、刚度、阻尼 setter。"""
from .episode import six

ACTIVE_JOINTS=('finger11','finger12','finger21','finger31','finger41','finger51')
MOTOR_IDS=(1,2,3,4,5,6)  # 厂家0是广播；数组下标0..5不可直接作为电机ID。

class ForcePositionAdapter:
    def __init__(self,sdk,config,authorize):
        self.sdk=sdk;self.config=config;self.authorize=authorize
    def snapshot(self):
        # 仅取 SDK 缓存，不调用 initial/config_map/enable/setter。
        return {'position_eq':[self.sdk.get_now_position(i) for i in MOTOR_IDS],
                'angle_deg':[self.sdk.get_now_angle(i) for i in MOTOR_IDS],
                'current_raw':[self.sdk.get_now_current(i) for i in MOTOR_IDS],
                'current_unit':self.config.get('current_unit'), 'torque_Nm':None,
                'force_N':None,'tactile':None,'model':self.config['hardware_model']}
    def command(self,position_eq,current_limit_raw):
        q=six(position_eq);limits=six(current_limit_raw);cfg=self.config
        if (q<0).any() or (q>10000).any():raise ValueError('hand position limit')
        if not cfg.get('motion_approved') or not cfg.get('motion_evidence'):raise RuntimeError('hand motion gate not reviewed')
        if not cfg.get('current_unit_evidence') or not cfg.get('current_unit'):raise RuntimeError('current units unresolved')
        bound=cfg.get('current_limit_raw')
        if bound is None or (limits<0).any() or (limits>six(bound)).any():raise ValueError('current limit unresolved/exceeded')
        self.authorize('hand_command')
        # 本地厂家 Python 包成功返回 None，失败抛异常；不自动上电/使能。
        if not cfg.get('vendor_return_contract_validated'):raise RuntimeError('vendor return contract unvalidated')
        try:
            for channel,i in enumerate(MOTOR_IDS):
                self.authorize('hand_command')
                for name,args in [('set_control_mode',(i,4)),('set_max_current',(i,int(limits[channel]))),('set_target_position',(i,int(q[channel])))]:
                    getattr(self.sdk,name)(*args)
            for i in MOTOR_IDS:
                self.authorize('hand_command')
                self.sdk.move_motors(i)
        except Exception:
            # 已进入命令事务后，任何调用失败或安全门撤销都尝试停止全部主动轴。
            # SDK停止成功仍不是机械停止确认；主站断链watchdog必须独立验收。
            self.last_stop_errors=[]
            for i in MOTOR_IDS:
                try:self.sdk.stop_motors(i)
                except Exception as exc:self.last_stop_errors.append(repr(exc))
            raise
