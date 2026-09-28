"""V1.5 动态采样执行适配器：共享只读核心，沿用 JAKA 深度2运动队列。

规划与执行解耦：控制柜执行时，PC 可以分次准备下一段；只有控制柜明确空出
预排槽时才提交。参考曲线/逆解采样不代表厂商实际圆滑路径的完整证明。
本模块不调用 servo、使能、清报警或滤波设置接口。
"""

from dataclasses import asdict
import math

from .adaptive_sampling import AdaptiveSampling
from .pose_blend_acceptance import RollingPoseQueue
from .sampled_follow import RejectedPath, measured
from .trajectory_horizon import PreviewProfile
from .intent_horizon import IntentHorizon
from .trajectory_reference import PoseSample
from .orientation_acceptance import _pose_rotation_error_deg


class AdaptivePoseQueue(RollingPoseQueue):
    """有界轨迹缓存 + 增量厂商逆解 + 已验证的队列状态机。

    计算密度与速度分开配置。过载时窗口限制新增运动；路径风险仍停止并要求
    松握重捕获。不跨过已接纳的折返。每轮检查许可，下发前再次读取最新许可。
    """

    def __init__(self, *args, policy=AdaptiveSampling(), position_radius_mm=1000.,
                 rotation_radius_deg=30., **kwargs):
        super().__init__(*args, **kwargs)
        self.policy = policy
        profile = PreviewProfile(
            speed_mm_s=self.settings.speed_mm_s,
            acceleration_mm_s2=self.settings.acceleration_mm_s2,
            angular_speed_deg_s=self.orientation_speed_deg_s,
            max_translation_mm=self.settings.segment_mm,
            max_rotation_deg=self.max_orientation_step_deg,
            input_timeout_s=self.settings.input_timeout_s,
        )
        self.horizon = IntentHorizon(
            self.robot.kine_inverse,
            self.limits,
            profile=profile,
            adaptive=True,
            policy=policy,
            clock=self.clock,
            position_radius_mm=position_radius_mm,
            rotation_radius_deg=rotation_radius_deg,
        )
        self.capture_id = self.last_input_s = None
        self.wait_release = False
        self.next_poll_s = 0.0
        self.plans = self.rejections = 0
        self.next_admission_report_s = 0.0

    def initialize(self):
        super().initialize()
        # 工作空间沿用会话启动中心，不能每次重握就把允许范围向外平移。
        self.horizon.workspace_origin = PoseSample.from_tcp(self.clock(), self.tail_tcp)
        self.emit(
            state="adaptive_queue_configuration",
            policy=asdict(self.policy),
            message="有界意图队列就绪；过载限制增量，不积压补追；队列深度2",
            scheduler="bounded_intent_v1",
            execution_mode="vendor_planned_waypoints",
            motion_method="linear_move_extend_ori",
            inverse_points_are_motion_commands=False,
            physical_C2_verified=False,
        )

    @property
    def planning_pending(self):
        return self.horizon.job is not None

    def _clear_reference(self):
        self.horizon.invalidate()
        self.capture_id = self.last_input_s = None

    def _abort(self, reason):
        # 先撤销尚未提交的工作，再要求厂商停止；停止失败不能继续使用缓存。
        self._clear_reference()
        super()._abort(reason)
        self.next_poll_s = 0.0
        self.last_status = None

    def _slot_open(self, state):
        """队列容量由控制柜读回判断，不能从PC完成逆解的速度推断。"""
        return not self.active or not (
            self.awaiting_busy
            or self.awaiting_queue_visibility
            or self.queued_was_visible
            or len(self.segments) != 1
            or state.inpos
            or state.active_queue <= 0
            or state.queue > 1
        )

    def tick(
        self, desired, *, permit, refresh_permit=None, received_s=None, capture_id=None,
        input_samples=None
    ):
        try:
            if self.done:
                return
            if not permit:
                if self.active or self.capture_id is not None:
                    self._abort("动态队列Grip/追踪许可已撤销")
                self.wait_release = False
                return
            if self.command_limit is not None and self.commands >= self.command_limit:
                # 有限段数验收仍由父状态机确认末段到位，不因动态采样多发一段。
                self._poll(True)
                return
            if self.wait_release or desired is None:
                return
            if refresh_permit is None:
                raise RuntimeError("动态真机队列必须提供下发前最新许可检查")
            now = self.clock()
            if (
                received_s is None
                or not math.isfinite(received_s)
                or not 0 <= now - received_s <= self.settings.input_timeout_s
            ):
                raise RejectedPath("动态目标时间已过期；不能用新心跳刷新旧目标")
            if not capture_id:
                raise RejectedPath("动态目标缺少Grip捕获标识")
            # 20ms状态轮询与逐点规划分开。SDK仍由同一线程串行调用。
            if now >= self.next_poll_s or self.last_status is None:
                self._poll(True)
                self.next_poll_s = self.clock() + 0.02
            if self.capture_id != capture_id:
                if self.active:
                    raise RejectedPath("运动期间捕获标识改变；停止后重新握持")
                self.last_status.require_idle_safe()
                joints, tcp = measured(self.robot)
                first_s = input_samples[0][1] if input_samples else received_s
                self.horizon.reset(PoseSample.from_tcp(first_s, tcp), joints)
                self.capture_id, self.last_input_s = capture_id, received_s
                self.tail_joints, self.tail_tcp = joints, tcp
                if not input_samples:
                    return
                self.last_input_s = first_s
            if input_samples:
                # 映射已在单SDK所有者线程按接收顺序完成。只做有界接纳；
                # 整批输入之后才前进一步逆解，避免每帧附加一次SDK调用。
                for tcp_sample, sample_s in input_samples:
                    if sample_s > self.last_input_s:
                        self.horizon.push(PoseSample.from_tcp(sample_s, tcp_sample), sample_s)
                        self.last_input_s = sample_s
            elif received_s != self.last_input_s:
                self.horizon.push(PoseSample.from_tcp(received_s, desired), received_s)
                self.last_input_s = received_s
            self.horizon.advance(permit=True)
            if self.clock() >= self.next_admission_report_s:
                self.next_admission_report_s = self.clock() + .25
                self.emit(state="intent_admission", planning=self.horizon.snapshot(),
                          message=("跟随受限：未接纳增量不补追，松握可重新对齐"
                                   if self.horizon.admission_scale < .999 else "正常接纳手柄轨迹"))
            if self.horizon.ready is None:
                return

            # 真实剩余执行时间当前SDK状态未提供，不能把路程/速度估计冒充读回值。
            # 已提前备好一段，发送时采用经过现场验证的“预排槽已空”状态条件。
            # 明确满槽时沿用20ms轮询节拍；不能在每次空转时重复读整套反馈。
            if not self._slot_open(self.last_status):
                return
            state = self._poll(True)
            self.next_poll_s = self.clock() + 0.02
            if not self._slot_open(state):
                return
            ready = self.horizon.ready
            # 不按候选墙钟年龄判断断流：后续仍检查最新输入许可、Grip epoch、
            # 实际起点以及已提交尾端。等待控制柜空槽不会触发重新握持。
            if not self.active:
                state.require_idle_safe()
                joints, tcp = measured(self.robot)
                # 规划期间机器人若被外部移动，不能将旧起点的检查套到新起点上。
                if (
                    max(
                        abs(math.degrees(a - b))
                        for a, b in zip(joints, ready.start_joints)
                    )
                    > 0.2
                    or math.dist(tcp[:3], ready.start.xyz) > 0.5
                    or _pose_rotation_error_deg(tcp, ready.start.tcp(tcp[3:])) > 0.1
                ):
                    raise RejectedPath("动态规划起点与实测不一致；需要重新捕获")
            elif (
                max(abs(a - b) for a, b in zip(self.tail_joints, ready.start_joints))
                > 1e-8
            ):
                raise RuntimeError("待提交逆解与控制柜已提交尾端不一致")
            if not refresh_permit():
                self._abort("动态下发前Grip/追踪/界面许可撤销")
                self.wait_release = True
                return
            plan = self.horizon.take_when_slot_open(expected_epoch=self.horizon.epoch)
            if plan is None:
                raise RejectedPath("动态候选已失效；需要重新捕获")
            # 控制柜参数保持原档位。圆滑半径先沿用已验证设置；实际过渡曲线需
            # 现场反馈验证，不能从稀疏采样点直接宣称速度/加速度处处连续。
            target = plan.command_tcp
            tolerance = (
                0.0
                if self.command_limit is not None
                and self.commands + 1 == self.command_limit
                else self.blend_tolerance_mm
            )
            self._dispatch_checked(
                plan.start_joints,
                plan.start.tcp(self.tail_tcp[3:]),
                target,
                plan.solutions,
                tolerance,
            )
            self.plans += 1
            self.emit(
                state="adaptive_queue_plan",
                target=target,
                inverse_calls=plan.inverse_calls,
                subdivisions=plan.subdivisions,
                avoided_endpoint_calls=plan.avoided_endpoint_calls,
                checked_fractions=plan.fractions,
                checked_joints=plan.solutions,
                planning=self.horizon.snapshot(),
                preparation_s=plan.preparation_s,
                completion_reason=plan.completion_reason,
                source_end_s=plan.target.t,
                command_index=self.commands,
                execution_mode="vendor_planned_waypoints",
                check_point_count=len(plan.solutions),
                motion_command_count=1,
            )
        except RejectedPath as error:
            self.rejections += 1
            detail = self.horizon.last_failure_context or self.horizon.snapshot()
            self.wait_release = True
            # 拒绝数值必须先落盘；后面的厂商停止调用也可能抛异常。
            self.emit(state="adaptive_queue_rejected", message=str(error),
                      planning=detail, rejections=self.rejections)
            self._abort(str(error))
            self.emit(
                state="adaptive_queue_paused",
                message=str(error),
                planning=detail,
                stop_confirmed=self.stop_confirmed,
                rejections=self.rejections,
            )
        except Exception as error:
            self.wait_release = True
            self._abort(f"动态队列异常：{error}；已撤销所有待提交轨迹")
            raise

    def shutdown(self):
        self._clear_reference()
        return super().shutdown()
