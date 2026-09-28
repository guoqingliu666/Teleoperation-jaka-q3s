"""V1.5 下一代外围参考轨迹：时间、位置和旋转共用一条进度。

这里没有机器人模型、逆解器、SDK 连接或运动指令。五次 Hermite 多项式只
构造手柄的 C2 参考曲线；归一化四元数表达姿态，不能把它等同于控制柜实际
执行的曲线。实际轨迹仍须由 JAKA 规划，并核查厂商圆滑过渡的执行语义。

为什么不只存最后一个姿态：一圈旋转的终点与起点相同，丢掉中间采样就丢了
转圈意图。这里保持有序的四元数路径；投影到工具局部 Z 的增量只作拧转诊断，
不是 J6 的命令角度，也不允许借取模绕过机械限位。
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from collections import deque
import math

from .quest_vr_input import quaternion, quaternion_matrix, matrix_rpy


def dot(a, b):
    return sum(x * y for x, y in zip(a, b, strict=True))


def qmul(a, b):
    """不归一化的 Hamilton 乘法；导数四元数不能当作单位姿态归一化。"""
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def rpy_quaternion(rpy):
    x, y, z = (v / 2 for v in rpy)
    sx, sy, sz, cx, cy, cz = math.sin(x), math.sin(y), math.sin(z), math.cos(x), math.cos(y), math.cos(z)
    return quaternion((sx*cy*cz-cx*sy*sz, cx*sy*cz+sx*cy*sz,
                       cx*cy*sz-sx*sy*cz, cx*cy*cz+sx*sy*sz))


def angle_deg(a, b):
    # atan2 在接近零角度时比 acos(dot) 稳定。
    d = qmul(qconj(a), b)
    return math.degrees(2 * math.atan2(math.hypot(*d[:3]), abs(d[3])))


def slerp(a, b, u):
    if dot(a, b) < 0:
        b = tuple(-v for v in b)
    theta = math.acos(max(-1., min(1., dot(a, b))))
    if theta < 1e-7:
        return quaternion(tuple((1-u)*x+u*y for x, y in zip(a, b)))
    return quaternion(tuple((math.sin((1-u)*theta)*x + math.sin(u*theta)*y)
                            / math.sin(theta) for x, y in zip(a, b)))


def distance_to_segment(p, a, b):
    d = tuple(y-x for x,y in zip(a,b))
    u = max(0., min(1., dot(tuple(x-y for x,y in zip(p,a)),d)/max(dot(d,d),1e-30)))
    return math.dist(p,tuple(x+u*y for x,y in zip(a,d)))


def chord_progress_interval(point, start, end, position_error_mm=1., rotation_error_deg=.25):
    """求同一六维进度 u 的可行区间，而非假设手柄匀速。

    平移球与线段相交给出一个区间；姿态沿短弧 SLERP 的球冠给出另一区间。
    取交集，保证不能为了压点而让位置走到一半、姿态却已走到终点。
    只用于小于90°的候选弦；不是机械臂轨迹或碰撞证明。
    """
    if (not math.isfinite(position_error_mm) or position_error_mm <= 0
            or not math.isfinite(rotation_error_deg) or not 0 < rotation_error_deg < 90):
        raise ValueError("弦误差预算无效")
    d = tuple(y-x for x,y in zip(start.xyz,end.xyz))
    r = tuple(x-y for x,y in zip(point.xyz,start.xyz))
    length2 = dot(d,d)
    low, high = 0., 1.
    if length2 < 1e-20:
        if math.hypot(*r) > position_error_mm:
            return None
    else:
        center = dot(r,d)/length2
        perpendicular2 = sum((x-center*y)**2 for x,y in zip(r,d))
        residual = position_error_mm**2-perpendicular2
        if residual < -1e-12:
            return None
        half = math.sqrt(max(0.,residual)/length2)
        low,high = max(low,center-half),min(high,center+half)
    a,b,p = start.q,end.q,point.q
    if dot(a,b) < 0: b = tuple(-x for x in b)
    if dot(a,p) < 0: p = tuple(-x for x in p)
    theta = math.acos(max(-1.,min(1.,dot(a,b))))
    if theta >= math.pi/4:
        return None  # 大弧保留中间采样，不能靠四元数符号折叠。
    if theta < 1e-7:
        # 近零弧的解析基退化。端点均在预算内是保守的充分条件。
        if max(angle_deg(a,p),angle_deg(b,p)) > rotation_error_deg:
            return None
    else:
        tangent = tuple((y-math.cos(theta)*x)/math.sin(theta) for x,y in zip(a,b))
        A,B = dot(p,a),dot(p,tangent)
        rho = math.hypot(A,B)
        threshold = math.cos(math.radians(rotation_error_deg)/2)
        if rho < threshold-1e-14:
            return None
        half = math.acos(max(-1.,min(1.,threshold/max(rho,1e-30))))
        center = math.atan2(B,A)
        low,high = max(low,(center-half)/theta),min(high,(center+half)/theta)
    if low > high+1e-10:
        return None
    return (max(0.,min(1.,low)),max(0.,min(1.,max(low,high))))


def ordered_chord_fits(points, start, end, position_error_mm=1., rotation_error_deg=.25):
    """存在单调的共同进度才允许合并；保留超过偏差预算的折返和完整转圈。"""
    lower = 0.
    for point in points:
        interval = chord_progress_interval(point,start,end,position_error_mm,rotation_error_deg)
        if interval is None:
            return False
        lower = max(lower,interval[0])
        if lower > interval[1]+1e-10:
            return False
    return True


@dataclass(frozen=True)
class PoseSample:
    """同一 Grip 会话中的映射后 TCP 参考；秒、毫米、XYZW。"""
    t: float
    xyz: tuple[float, float, float]
    q: tuple[float, float, float, float]

    def __post_init__(self):
        if len(self.xyz) != 3 or not all(math.isfinite(x) for x in (self.t, *self.xyz)):
            raise ValueError("参考样本的时间/位置必须是有限数")
        object.__setattr__(self, "xyz", tuple(float(x) for x in self.xyz))
        object.__setattr__(self, "q", quaternion(self.q))

    @classmethod
    def from_tcp(cls, t, tcp):
        if len(tcp) != 6 or not all(math.isfinite(x) for x in tcp):
            raise ValueError("TCP 必须是六个有限数")
        return cls(t, tuple(tcp[:3]), rpy_quaternion(tcp[3:]))

    def tcp(self, near_rpy=(0., 0., 0.)):
        angles = matrix_rpy(quaternion_matrix(self.q))
        # 只选连续的 RPY 表达；从不对厂商返回的关节角取模。
        return self.xyz + tuple(x + round((n-x)/(2*math.pi))*2*math.pi
                                for x, n in zip(angles, near_rpy))


class RotationHistory:
    """保存连续旋转弧长和工具局部 Z 拧转增量；异常后必须重新捕获。

    不会将超过门槛的样本当作下一帧基准；否则下一帧可能绕过突变检查。
    max_step/max_gap 是输入有效性门槛，不是机器人速度或 J6 限位。
    """
    def __init__(self, max_step_deg=20., max_gap_s=.15):
        if not 0 < max_step_deg < 180 or not 0 < max_gap_s <= 1:
            raise ValueError("旋转历史门槛无效")
        self.max_step, self.max_gap = max_step_deg, max_gap_s
        self.reset()

    def reset(self):
        self.previous = None
        self.arc_deg = self.twist_deg = 0.
        self.blocked = False
        self.last_failure = None

    def push(self, sample):
        if self.blocked:
            raise ValueError("旋转历史已失效，需释放 Grip 后重新捕获")
        if self.previous is not None:
            previous = self.previous
            dt = sample.t - previous.t
            step = angle_deg(previous.q, sample.q)
            if not 0 < dt <= self.max_gap or step > self.max_step:
                self.blocked = True
                # 保留被拒绝的原始证据，不能只写一条含三个可能原因的提示。
                # 不更新 previous；下一帧不能借异常点重置基准绕过保护。
                reason = ("时间倒退或重复" if dt <= 0 else
                          "输入采样间隔超限" if dt > self.max_gap else "姿态单帧变化超限")
                self.last_failure = dict(reason=reason, dt_s=dt, step_deg=step,
                    maximum_gap_s=self.max_gap, maximum_step_deg=self.max_step,
                    previous_t=previous.t, rejected_t=sample.t,
                    previous_q=previous.q, rejected_q=sample.q)
                raise ValueError(f"{reason}：dt={dt:.6f}s，姿态变化={step:.3f}°；禁止补追未知旋转")
            if dot(previous.q, sample.q) < 0:
                sample = replace(sample, q=tuple(-x for x in sample.q))
            local = qmul(qconj(previous.q), sample.q)
            self.arc_deg += angle_deg(previous.q, sample.q)
            self.twist_deg += math.degrees(2 * math.atan2(local[2], local[3]))
        self.previous = sample
        return sample


def adaptive_knots(samples, *, position_error_mm=1., rotation_error_deg=.25,
                   max_span_s=.12, max_arc_deg=15.):
    """按位置/姿态弦误差保留转弯，并以时间和累计旋转限制简化跨度。

    不是只比较首尾姿态：即使一圈回到同一姿态，累计弧长也禁止合并成零转动。
    此函数作用于一个已确认连续的短窗口，不能跨 Grip、追踪丢失或断流调用。
    """
    if any(not math.isfinite(x) or x <= 0 for x in
           (position_error_mm, rotation_error_deg, max_span_s, max_arc_deg)):
        raise ValueError("采样误差和跨度必须为正")
    points = list(samples)
    if any(b.t <= a.t for a, b in zip(points, points[1:])):
        raise ValueError("样本时间必须严格递增")
    if len(points) < 3:
        return points
    selected = [points[0]]
    start = 0
    end = 2
    while end < len(points):
        a, b = points[start], points[end]
        arc = sum(angle_deg(points[j-1].q, points[j].q) for j in range(start+1, end+1))
        keep = b.t-a.t > max_span_s or arc > max_arc_deg
        if not keep:
            keep = not ordered_chord_fits(points[start+1:end],a,b,
                                         position_error_mm,rotation_error_deg)
        if keep:
            start = end-1
            selected.append(points[start])
        end += 1
    selected.append(points[-1])
    return selected


@dataclass(frozen=True)
class ReferenceState:
    pose: PoseSample
    velocity: tuple
    acceleration: tuple
    angular_velocity: tuple  # 基坐标系，rad/s
    angular_acceleration: tuple  # rad/s²


class QuinticSpan:
    """七维 XYZ+四元数分量五次曲线，共享端点位置、一阶/二阶导数。

    单位四元数曲线由非零四维多项式归一化得到。归一化是光滑映射，因此
    非零条件下跨段保持 C2；下面同时计算其解析导数，不用画面平滑代替验证。
    """
    def __init__(self, left, right, left_velocity, right_velocity):
        self.left, self.right = left, right
        h = right.t-left.t
        if h <= 0:
            raise ValueError("曲线时间必须递增")
        self.h = h
        self.coefficients = []
        for p0, p1, v0, v1 in zip((*left.xyz, *left.q), (*right.xyz, *right.q),
                                  left_velocity, right_velocity, strict=True):
            c0, c1 = p0, h*v0
            dp, dv = p1-c0-c1, h*v1-c1
            self.coefficients.append((c0, c1, 0., 10*dp-4*dv, -15*dp+7*dv, 6*dp-3*dv))

    def controls(self, derivative=0):
        """幂基转 Bernstein 控制点。凸包界限覆盖整段，不只检查有限采样点。"""
        degree = 5-derivative
        columns = []
        for c in self.coefficients:
            values = [c[i]*math.factorial(i)/math.factorial(i-derivative)/self.h**derivative
                      for i in range(derivative,6)]
            columns.append([sum(math.comb(k,i)/math.comb(degree,i)*values[i]
                                for i in range(k+1)) for k in range(degree+1)])
        return [tuple(column[k] for column in columns) for k in range(degree+1)]

    def bounds(self):
        """保守全段几何/导数上界；用于参考时间缩放，不是控制柜动力学保证。"""
        c, v, a = self.controls(), self.controls(1), self.controls(2)
        qmin = min(dot(p[3:],self.left.q) for p in c)
        if qmin <= .5:
            raise ValueError("无法证明四元数曲线远离零点")
        qv, qa = max(math.hypot(*p[3:]) for p in v), max(math.hypot(*p[3:]) for p in a)
        deviation = max(distance_to_segment(p[:3],self.left.xyz,self.right.xyz) for p in c)
        qdev = max(distance_to_segment(p[3:],self.left.q,self.right.q) for p in c)
        return {
            "position_deviation_mm": deviation,
            "orientation_deviation_deg": math.degrees(2*math.asin(min(1.,qdev/qmin))),
            "speed_mm_s": max(math.hypot(*p[:3]) for p in v),
            "acceleration_mm_s2": max(math.hypot(*p[:3]) for p in a),
            "angular_speed_deg_s": math.degrees(2*qv/qmin),
            "angular_acceleration_deg_s2": math.degrees(4*qa/qmin+12*qv*qv/(qmin*qmin)),
        }

    def evaluate(self, t):
        if not self.left.t-1e-10 <= t <= self.right.t+1e-10:
            raise ValueError("不能外推手柄轨迹")
        u = max(0., min(1., (t-self.left.t)/self.h))
        p, v, a = [], [], []
        for c in self.coefficients:
            p.append(sum(c[i]*u**i for i in range(6)))
            v.append(sum(i*c[i]*u**(i-1) for i in range(1, 6))/self.h)
            a.append(sum(i*(i-1)*c[i]*u**(i-2) for i in range(2, 6))/self.h**2)
        raw, d, dd = p[3:], v[3:], a[3:]
        norm = math.hypot(*raw)
        if norm < .5:
            raise ValueError("四元数曲线接近零，拒绝归一化放大")
        dn = dot(raw, d)/norm
        ddn = (dot(d, d)+dot(raw, dd)-dn*dn)/norm
        q = tuple(x/norm for x in raw)
        dq = tuple(y/norm-x*dn/norm**2 for x, y in zip(raw, d))
        ddq = tuple(z/norm-2*y*dn/norm**2-x*ddn/norm**2+2*x*dn*dn/norm**3
                    for x, y, z in zip(raw, d, dd))
        omega = qmul(dq, qconj(q))
        alpha = qmul(ddq, qconj(q))
        # dq*conj(dq) 为实数，不影响角加速度的虚部。
        return ReferenceState(PoseSample(t, tuple(p[:3]), q), tuple(v[:3]), tuple(a[:3]),
                              tuple(2*x for x in omega[:3]), tuple(2*x for x in alpha[:3]))


class C2Reference:
    """一个连续 Grip 窗口的候选参考曲线；不是厂商已执行轨迹。

    中间结点使用共享中心差分速度，二阶导数统一为零，避免每个结点强制停速。
    首尾静止用于离线有限窗口。在线滚动不能重复拼接这种零速端点，必须冻结
    已承诺结点的导数并使用前后文；本类不自动接入运动发送端。
    """
    def __init__(self, samples, *, position_deviation_mm=1., orientation_deviation_deg=.25):
        if any(not math.isfinite(x) or x <= 0 for x in (position_deviation_mm,orientation_deviation_deg)):
            raise ValueError("曲线偏差预算必须为正数")
        history = RotationHistory(max_step_deg=179., max_gap_s=1.)
        self.points = [history.push(p) for p in samples]
        if len(self.points) < 2:
            raise ValueError("参考曲线至少需要两个样本")
        velocities = [(0.,)*7]
        for before, current, after in zip(self.points, self.points[1:], self.points[2:]):
            h0, h1 = current.t-before.t, after.t-current.t
            # 非均匀时间戳的三点导数，不能假设每帧恰好 1/60 秒。
            values = []
            for x, y, z in zip((*before.xyz, *before.q), (*current.xyz, *current.q),
                               (*after.xyz, *after.q)):
                values.append((h1*(y-x)/h0 + h0*(z-y)/h1)/(h0+h1))
            velocities.append(tuple(values))
        velocities.append((0.,)*7)
        self.tangent_reductions = 0
        for _ in range(32):
            self.spans = [QuinticSpan(a, b, va, vb) for a, b, va, vb in zip(
                self.points, self.points[1:], velocities, velocities[1:])]
            reduce = set()
            for i,span in enumerate(self.spans):
                try:
                    bounds = span.bounds()
                    good = (bounds["position_deviation_mm"] <= position_deviation_mm
                            and bounds["orientation_deviation_deg"] <= orientation_deviation_deg)
                except ValueError:
                    good = False
                if not good:
                    reduce.update((i,i+1))
            if not reduce:
                break
            # 同一结点的左右两段同时更新，保持C2，不能逐段各改各的速度。
            for i in reduce:
                velocities[i] = tuple(x*.5 for x in velocities[i])
                self.tangent_reductions += 1
        else:
            raise ValueError("无法满足参考曲线偏差预算")

    def derivative_bounds(self):
        bounds = [span.bounds() for span in self.spans]
        return {key:max(b[key] for b in bounds) for key in bounds[0]}

    def required_time_scale(self, *, speed_mm_s, acceleration_mm_s2,
                            angular_speed_deg_s, angular_acceleration_deg_s2):
        """统一时间伸缩的充分比例；保留曲线几何和C2，不在每段重新零速起步。

        这是有限窗口参考的保守缩放；在线若要改变缩放率，还需约束缩放率本身
        的导数。不能把该浮点数直接当作SDK的每段速度或自动降低安全门槛。
        """
        limits=(speed_mm_s,acceleration_mm_s2,angular_speed_deg_s,angular_acceleration_deg_s2)
        if any(not math.isfinite(x) or x <= 0 for x in limits):
            raise ValueError("时间缩放需要明确的有限正速度/加速度预算")
        b=self.derivative_bounds()
        return max(1., b["speed_mm_s"]/speed_mm_s,
                   math.sqrt(b["acceleration_mm_s2"]/acceleration_mm_s2),
                   b["angular_speed_deg_s"]/angular_speed_deg_s,
                   math.sqrt(b["angular_acceleration_deg_s2"]/angular_acceleration_deg_s2))

    def continuity_errors(self):
        """解析端点验证位置/速度/加速度和姿态/角速度/角加速度，单位不混合。"""
        errors = dict(position_mm=0., velocity_mm_s=0., acceleration_mm_s2=0.,
                      orientation_deg=0., angular_velocity_rad_s=0., angular_acceleration_rad_s2=0.)
        for a, b in zip(self.spans, self.spans[1:]):
            x, y = a.evaluate(a.right.t), b.evaluate(b.left.t)
            values = (math.dist(x.pose.xyz, y.pose.xyz), math.dist(x.velocity, y.velocity),
                      math.dist(x.acceleration, y.acceleration), angle_deg(x.pose.q, y.pose.q),
                      math.dist(x.angular_velocity, y.angular_velocity),
                      math.dist(x.angular_acceleration, y.angular_acceleration))
            errors = {key: max(old, new) for (key, old), new in zip(errors.items(), values)}
        return errors


def knot_velocity(before, current, after):
    """同一个结点始终使用同一组前后文，已经交给执行端的导数不得重算。"""
    h0, h1 = current.t-before.t, after.t-current.t
    return tuple((h1*(y-x)/h0+h0*(z-y)/h1)/(h0+h1)
                 for x, y, z in zip((*before.xyz, *before.q), (*current.xyz, *current.q),
                                    (*after.xyz, *after.q)))


class StreamingReference:
    """一帧前瞻的在线 C2 曲线生成器；不会把多个零速小窗口拼在一起。

    每个样本最多参与两次局部计算，空间恒定。需要消费者及时接走返回的 span。
    释放 Grip 用 reset 丢弃未承诺曲线；不能 flush 后拿去追赶已释放的手柄。
    finish 只适用于离线轨迹结束/仍持有许可时的末端分析，不发送停机命令。
    """
    def __init__(self, *, position_deviation_mm=1., orientation_deviation_deg=.25):
        if (not math.isfinite(position_deviation_mm) or position_deviation_mm <= 0
                or not math.isfinite(orientation_deviation_deg) or orientation_deviation_deg <= 0):
            raise ValueError("在线偏差预算必须为有限正数")
        self.position_deviation_mm=position_deviation_mm
        self.orientation_deviation_deg=orientation_deviation_deg
        self.history = RotationHistory()
        self.pending = deque()
        self.left_velocity = (0.,)*7
        self.finished = False
        self.tangent_reductions = 0

    def reset(self):
        self.history.reset()
        self.pending.clear()
        self.left_velocity = (0.,)*7
        self.finished = False
        self.tangent_reductions = 0

    def _within_budget(self, span):
        try:
            b=span.bounds()
            return (b["position_deviation_mm"] <= self.position_deviation_mm
                    and b["orientation_deviation_deg"] <= self.orientation_deviation_deg)
        except ValueError:
            return False

    def push(self, sample):
        if self.finished:
            raise ValueError("已结束的轨迹必须重新捕获")
        sample = self.history.push(sample)
        self.pending.append(sample)
        if len(self.pending) < 3:
            return None
        left, right, future = self.pending
        velocity = knot_velocity(left, right, future)
        for _ in range(40):
            span = QuinticSpan(left, right, self.left_velocity, velocity)
            # 零端加速度五次Bezier的前三个控制点只取决于左结点的导数，
            # 后三个只取决于右结点的导数。提交right导数前，同时验证它对
            # 下一段(right,future)的影响；未来只改另一端，不能反改已提交导数。
            outgoing=QuinticSpan(right,future,velocity,(0.,)*7)
            if self._within_budget(span) and self._within_budget(outgoing):
                break
            velocity=tuple(x*.5 for x in velocity)
            self.tangent_reductions+=1
        else:
            self.finished=True
            raise ValueError("在线前后段无法同时满足偏差预算；需要重新捕获")
        self.left_velocity = velocity
        self.pending.popleft()
        return span

    def finish(self):
        self.finished = True
        if len(self.pending) != 2:
            return None
        left, right = self.pending
        self.pending.clear()
        span=QuinticSpan(left, right, self.left_velocity, (0.,)*7)
        if not self._within_budget(span):
            raise ValueError("在线尾段无法满足偏差预算")
        return span
