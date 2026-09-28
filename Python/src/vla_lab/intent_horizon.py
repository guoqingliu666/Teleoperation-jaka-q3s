"""有界运动意图窗口：把新鲜输入、计算工作和待执行路径分别管理。

不逐帧补追无限历史。负荷过大时按同一比例缩小平移和旋转增量，明确记录
未接纳的运动；这些增量不会在停手后偷偷补发。Grip 重捕获恢复一比一基准。
逆解仍全部来自厂商；本模块只能交付已检查候选，没有机器人运动接口。
"""

import math

from .adaptive_inverse import AdaptiveInverse
from .trajectory_horizon import TimeHorizon
from .trajectory_reference import PoseSample, angle_deg, qmul, qconj, slerp


class IntentHorizon(TimeHorizon):
    """以路径工作量约束缓存，而不是按最老输入的年龄自动暂停。

    pending_seconds 是路程/配置速度的名义工作量，绝不是控制柜剩余执行时间。
    其上限仅用于输入接纳；真实下发仍须由执行器检查队列、反馈和最新许可。
    固定的点数上限另行限制曲折、噪声输入，防止零长度小点耗尽内存。
    """

    def __init__(self, *args, position_radius_mm=1000.0,
                 rotation_radius_deg=30.0, **kwargs):
        super().__init__(*args, **kwargs)
        if not (math.isfinite(position_radius_mm) and position_radius_mm > 0
                and math.isfinite(rotation_radius_deg) and 0 < rotation_radius_deg <= 180):
            raise ValueError("意图窗口需要有效的位置和姿态范围")
        self.position_radius_mm = position_radius_mm
        self.rotation_radius_deg = rotation_radius_deg
        self.capacity_s = .35
        self.capacity_points = 128
        self.origin = self.raw_previous = self.admitted = None
        self.workspace_origin = None
        self.limited_frames = self.discarded_mm = self.discarded_deg = 0
        self.admission_scale = 1.0
        self.input_failure = None

    def reset(self, anchor, joints):
        super().reset(anchor, joints)
        self.origin = self.workspace_origin or anchor
        self.raw_previous = self.admitted = anchor
        self.limited_frames = self.discarded_mm = self.discarded_deg = 0
        self.admission_scale = 1.0
        self.input_failure = None

    def _cost(self, a, b):
        return max(math.dist(a.xyz, b.xyz) / self.profile.speed_mm_s,
                   angle_deg(a.q, b.q) / self.profile.angular_speed_deg_s)

    def pending_seconds(self):
        points = [self.anchor] + [p for p, _ in self.samples]
        return sum(self._cost(a, b) for a, b in zip(points, points[1:]))

    def snapshot(self):
        result = super().snapshot()
        result.update(scheduler="bounded_intent_v1", pending_nominal_s=self.pending_seconds()
                      if self.anchor is not None else 0.0,
                      admission_scale=self.admission_scale,
                      limited_frames=self.limited_frames, discarded_translation_mm=self.discarded_mm,
                      discarded_rotation_deg=self.discarded_deg,
                      admitted_tcp=self.admitted.tcp() if self.admitted else None,
                      sdk_compute_s=getattr(self.job, "sdk_elapsed_s", None))
        result["input_failure"] = self.input_failure
        result["inverse_risk"] = getattr(self.job, "risk_detail", None)
        return result

    def push(self, sample, received_s):
        """接纳有界的新增运动。原始时间不改写，未接纳增量不进入缓存。"""
        if self.blocked or self.anchor is None:
            raise RuntimeError("意图窗口需要重新捕获")
        now = self.clock()
        previous = self.raw_previous
        # 分别记录接收新鲜度、处理样本间隔和几何变化。SDK阻塞后漏处理样本，
        # 与手柄真的跳动不是同一原因；没有中间帧证据时仍停止，不放宽门槛。
        gap = sample.t-previous.t
        turn = angle_deg(previous.q, sample.q)
        distance = math.dist(previous.xyz, sample.xyz)
        reason = None
        if not math.isfinite(received_s) or not 0 <= now-received_s <= self.profile.input_timeout_s:
            reason = "接收数据过期或接收时间无效"
        elif not 0 <= received_s-sample.t <= self.profile.input_timeout_s:
            reason = "源样本时间无效或过期"
        elif gap <= 0:
            reason = "源样本时间未递增"
        elif gap > self.profile.input_timeout_s:
            reason = "相邻已处理样本时间间隔过大"
        elif turn > 20:
            reason = "相邻输入姿态变化超过20°"
        elif distance > 100:
            reason = "相邻输入位置变化超过100mm"
        if reason:
            self.input_failure = dict(reason=reason, processed_gap_s=gap,
                receive_age_s=now-received_s, rotation_step_deg=turn,
                translation_step_mm=distance)
            self.invalidate()
            raise RuntimeError(f"{reason}；需重新捕获")
        self.latest_received_s, self.last_sample = received_s, sample
        self.raw_previous = sample
        translation = math.dist(previous.xyz, sample.xyz)
        rotation = angle_deg(previous.q, sample.q)
        if translation < 1e-8 and rotation < 1e-7:
            return  # 新心跳保持有效，但静止不生成零长运动段。
        cost = self._cost(previous, sample)
        free = max(0.0, self.capacity_s-self.pending_seconds())
        # 在缓存用到一半后逐步降低增益，留出用于转折和 SDK 抖动的余量。
        # 恢复增益也有限速，避免队列刚空出就突然接纳大幅运动。
        desired_scale = min(1.0, free/(self.capacity_s*.5))
        scale = min(desired_scale, self.admission_scale + (sample.t-previous.t)*4,
                    free/max(cost, 1e-12))
        if len(self.samples) >= self.capacity_points:
            scale = 0.0
        self.admission_scale = max(0.0, scale)
        position = tuple(a + scale*(b-c) for a,b,c in
                         zip(self.admitted.xyz, sample.xyz, previous.xyz))
        # 世界系旋转增量左乘参考姿态；平移和旋转用同一个缩放系数。
        delta = qmul(sample.q, qconj(previous.q))
        orientation = qmul(slerp((0.,0.,0.,1.), delta, scale), self.admitted.q)
        # 受限输入会产生操纵偏移，因此必须再检查接纳目标的全局活动范围，
        # 不能只依赖上游对原始黄色目标做过的范围裁剪。
        distance = math.dist(self.origin.xyz, position)
        if distance > self.position_radius_mm:
            position = tuple(a+(b-a)*self.position_radius_mm/distance
                             for a,b in zip(self.origin.xyz, position))
        turn = angle_deg(self.origin.q, orientation)
        if turn > self.rotation_radius_deg:
            orientation = slerp(self.origin.q, orientation, self.rotation_radius_deg/turn)
        accepted = PoseSample(sample.t, position, orientation)
        if scale < 1.-1e-9:
            self.limited_frames += 1
            self.discarded_mm += (1-scale)*translation
            self.discarded_deg += (1-scale)*rotation
        if (math.dist(self.admitted.xyz, accepted.xyz) > 1e-8
                or angle_deg(self.admitted.q, accepted.q) > 1e-7):
            self.samples.append((accepted, received_s))
        self.admitted = accepted

    def advance(self, *, permit):
        """每轮一次厂商逆解；等待反馈、排队不会把计算任务变成运动短段。"""
        if not permit:
            self.invalidate()
            return
        if self.blocked:
            raise RuntimeError("意图窗口已失效，需要重新捕获")
        if self.latest_received_s is None:
            return
        if self.clock()-self.latest_received_s > self.profile.input_timeout_s:
            self.invalidate()
            raise RuntimeError("手柄输入断流")
        self.pressure = "limited" if self.admission_scale < .999 else "normal"
        # 已检查候选依附于不可变的规划起点和 Grip epoch。正常等候控制柜槽位
        # 不等于输入断流，不能只按候选创建时间将其判废。
        if self.ready is not None:
            return
        try:
            if self.job is None:
                target = self._select_target()
                if target is None:
                    return
                self.job = AdaptiveInverse(self.inverse, self.anchor, target, self.joints,
                    self.limits, epoch=self.epoch, profile=self.profile,
                    policy=self.policy, clock=self.clock, cooperative=True)
            result = self.job.advance()
            if result is not None:
                self.costs.append(result.preparation_s)
                self.ready, self.job = result, None
        except Exception:
            self.invalidate()
            raise

    def _select_target(self):
        """平滑小动作按预计段时间合并，转折仍立即留点，静止末端有限等待。"""
        target = super()._select_target()
        if target is None or not self.samples:
            return target
        # 准备耗时只用来估算下一段应覆盖的时间，绝不当作控制柜反馈。
        desired_duration = min(.20, max(.08, 2*max(self.costs, default=.02)+.02))
        smooth_tail = target == self.samples[-1][0]
        waiting = self.clock()-self.samples[0][1]
        if smooth_tail and self._cost(self.anchor,target) < desired_duration and waiting < .16:
            return None
        return target

    def take_preview(self, remaining_execution_s, *, permit, expected_epoch):
        """只交付当前 epoch 完整检查结果；调用者负责控制柜槽位与起点复核。"""
        if not permit or expected_epoch != self.epoch:
            self.invalidate()
            return None
        if self.blocked or self.ready is None or remaining_execution_s is None:
            return None
        if not math.isfinite(remaining_execution_s) or remaining_execution_s < 0:
            raise ValueError("剩余执行时间无效")
        if self.latest_received_s is None or self.clock()-self.latest_received_s > self.profile.input_timeout_s:
            self.invalidate()
            return None
        if remaining_execution_s > self.replenishment_threshold_s():
            return None
        result, self.ready = self.ready, None
        self.anchor, self.joints = result.target, result.solutions[-1]
        oldest = self.samples[0][1] if self.samples else result.created_s
        while self.samples and self.samples[0][0].t <= result.source_target.t:
            self.samples.popleft()
        if result.target.t < result.source_target.t:
            self.samples.appendleft((result.source_target, oldest))
        return result
