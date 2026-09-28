"""V1.5 动态采样参数与关键点选择。

这里决定哪些手柄点可以合并，不计算机器人的关节解。位置和姿态共用同一
路径进度；折返和连续转圈必须保留。所有参数都是软件检查参数，不是机器人的
额定性能，也不能单独证明采样点之间或厂商圆滑路径上没有碰撞、奇异点。
"""

from dataclasses import asdict, dataclass
import math

from .trajectory_reference import angle_deg, ordered_chord_fits


@dataclass(frozen=True)
class AdaptiveSampling:
    """先用较疏的间隔检查，风险增大时只细分相应区间。

    max_* 是最终通过检查的相邻点最大间隔，不是机器人运动段长度。
    fine_* 只决定何时已充分细分；无法消除的风险仍会拒绝，不能靠细分放行。
    改大 max_* 需要重新做离线与只读对照，不会联动提高真机速度。
    """

    max_check_mm: float = 5.0
    max_check_deg: float = 1.0
    fine_check_mm: float = 0.25
    fine_check_deg: float = 0.05
    position_error_mm: float = 1.0
    orientation_error_deg: float = 0.25
    joint_curve_error_deg: float = 0.15
    joint_gain_deg: float = 1.5
    # 放大比高时，把相邻关节检查步长细化到此值即可；放大比本身不会随二分
    # 消失。它是检查密度，不是允许机器人每控制周期运动的角度。
    gain_resolved_step_deg: float = 0.25
    near_limit_deg: float = 8.0
    minimum_margin_deg: float = 3.0
    joint_step_deg: float = 3.0
    joint_segment_deg: float = 12.0
    soft_backlog_s: float = 0.2
    pause_backlog_s: float = 0.3
    max_calls: int = 256
    max_depth: int = 10

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError("动态采样参数必须为有限正数")
        if (
            self.fine_check_mm > self.max_check_mm
            or self.fine_check_deg > self.max_check_deg
            or self.minimum_margin_deg >= self.near_limit_deg
            or self.soft_backlog_s >= self.pause_backlog_s
            or self.max_check_deg >= 90
            or self.gain_resolved_step_deg >= self.joint_step_deg
        ):
            raise ValueError("动态采样间隔/余量/积压参数顺序不正确")
        if type(self.max_calls) is not int or type(self.max_depth) is not int:
            raise ValueError("逆解次数和细分深度必须是整数")


def select_keypoint(
    anchor, samples, *, max_translation_mm, max_rotation_deg, policy=AdaptiveSampling()
):
    """选取最长的合规前缀：平滑处向前合并，第一次偏离处留下转折。

    samples 是按原始时间排列的 PoseSample。始终对原始点检查，不能把已经
    压缩过的端点再次当原始轨迹，避免反复压缩时误差逐层累加。
    首点超过段长限制时仍返回首点，交给逆解任务按共同进度截短并保留残量。
    """
    if not samples:
        return None
    target = samples[0]
    for index, candidate in enumerate(samples):
        if candidate.t <= anchor.t:
            raise ValueError("关键点必须严格晚于当前规划起点")
        if (
            math.dist(anchor.xyz, candidate.xyz) > max_translation_mm
            or angle_deg(anchor.q, candidate.q) > max_rotation_deg
        ):
            break
        if not ordered_chord_fits(
            samples[:index],
            anchor,
            candidate,
            policy.position_error_mm,
            policy.orientation_error_deg,
        ):
            break
        target = candidate
    return target
