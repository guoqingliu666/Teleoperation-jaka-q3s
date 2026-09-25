"""②连续采样运行入口。默认仅说明；--shadow 只读；--live 才发送厂商短段运动。

界面用标准输入传心跳/停止，运行线程串行持有唯一 SDK 连接。
①单独运行时只读 JAKA；②启动前让①注销，随后②独占 SDK 并广播实测状态。
②结束后①恢复只读。目标显示与实测反馈始终分开。
SDK 调用耗时、状态转换、每个实际下发目标都写 JSONL，便于复查卡顿与停机原因。
"""
from __future__ import annotations

import argparse
import configparser
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
SESSION_LOG_ROOT = ROOT / "Validation" / "continuous_sampled_follow"
sys.path.insert(0, str(ROOT / "Python" / "src"))
from vla_lab.jaka_telemetry import SDK_DIRECTORY, load_sdk, validate_host
from vla_lab.quest_vr_input import QuestUdpReceiver
from vla_lab.sampled_follow import Settings, SampledFollower, check_health, checked
from vla_lab.sdk_trace import TraceFile, TracedSdk
from vla_lab.single_owner_feedback import SingleOwnerFeedback
from vla_lab.sdk_owner_lease import SdkOwnerLease, reject_legacy_bridge, request_readonly_handoff
from vla_lab.rotation_shadow import RotationShadow
from vla_lab.orientation_acceptance import OrientationAcceptance
from vla_lab.bounded_pose_follow import BoundedPoseFollower


class UiPermit:
    """失去窗口、管道 EOF、STOP 或心跳0.5秒超时都会撤销运动许可。"""
    def __init__(self):
        self.last = time.monotonic()
        self.closed = False
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        """在后台读 GUI 的标准输入，避免等待一行命令阻塞 SDK 主循环。"""
        try:
            for line in sys.stdin:
                if line.strip() == "HEARTBEAT":
                    self.last = time.monotonic()
                elif line.strip() == "STOP":
                    break
        finally:
            self.closed = True

    def valid(self):
        """只有管道仍开且最近半秒收到心跳，界面许可才算有效。"""
        return not self.closed and time.monotonic() - self.last < .5


def read_limits(path):
    """使用用户导出的控制柜配置作软件边界，记录哈希；不修改控制柜配置。

    这是配置快照；若现场修改限位需重新导出。本限制不替代控制器硬限位。
    """
    raw = path.read_bytes()
    config = configparser.ConfigParser(strict=False)
    config.read_string(raw.decode("utf-8-sig"))
    limits = [(float(config[f"JOINT_{i}"]["JOINT_MIN_LIMIT"]),
               float(config[f"JOINT_{i}"]["JOINT_MAX_LIMIT"])) for i in range(6)]
    if any(not math.isfinite(lo+hi) or lo >= hi for lo, hi in limits):
        raise ValueError("关节限位配置无效")
    return limits, hashlib.sha256(raw).hexdigest()


def main(argv=None):
    """单 SDK 所有者的完整会话：门槛检查 → 连接 → 输入/反馈循环 → 停止清理。

    `--shadow` 与 `--comm-check` 不发送运动；`--live` 还必须同时满足 GUI
    心跳、单连接显示和明确的受限验收档位。CLI 参数本身不能跳过这些门槛。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--live", action="store_true")
    mode.add_argument("--shadow", action="store_true")
    mode.add_argument("--comm-check", action="store_true", help="只读通信压力检查，不接收手柄，不做逆解或运动")
    mode.add_argument("--rotation-shadow", action="store_true",
                      help="30秒姿态只读影子：调用厂商逆解并显示目标姿态，绝不运动")
    parser.add_argument("--ui-heartbeat", action="store_true")
    parser.add_argument("--ui-protocol", type=int, default=0)
    parser.add_argument("--single-owner-display", action="store_true",
                        help="由本进程向 Unity 广播实测反馈；①会在交接后暂停只读连接")
    parser.add_argument("--one-segment-acceptance", action="store_true",
                        help="一次真机验收：最多一段5mm、5mm/s，到位或中止后结束")
    parser.add_argument("--two-segment-acceptance", action="store_true",
                        help="第二关：最多两段各10mm、10mm/s，均实测到位后自动结束")
    parser.add_argument("--five-segment-acceptance", action="store_true",
                        help="第三关：最多五段各10mm、15mm/s，总指令行程≤50mm")
    parser.add_argument("--ten-segment-acceptance", action="store_true",
                        help="第四关：最多十段各10mm、20mm/s，总指令行程≤100mm")
    parser.add_argument("--twenty-segment-acceptance", action="store_true",
                        help="第五关：最多二十段各10mm、30mm/s，总指令行程≤200mm")
    parser.add_argument("--bounded-continuous", action="store_true",
                        help="已验收后的受限连续位置跟随：半径≤200mm、速度≤30mm/s、最长600秒")
    parser.add_argument("--bounded-six-dof", action="store_true",
                        help="受限连续六维跟随：位置半径≤200mm、姿态±10°、每段≤10mm/1°")
    parser.add_argument("--expanded-six-dof", action="store_true",
                        help="扩展六维跟随：500mm、50mm/s、±30°、每段≤20mm/2°")
    parser.add_argument("--production-six-dof", action="store_true",
                        help="正式可调六维：200—1000mm、5—100mm/s、±10—30°，每段≤20mm/2°")
    parser.add_argument("--rotation-radius-deg", type=float, default=30)
    parser.add_argument("--orientation-speed-deg-s", type=float, default=10)
    parser.add_argument("--one-degree-orientation-acceptance", action="store_true",
                        help="一次≤1°纯姿态真机验收：1°/s、2°/s²，XYZ保持")
    parser.add_argument("--three-degree-orientation-acceptance", action="store_true",
                        help="三段累计≤3°纯姿态真机验收：每段≤1°、2°/s、4°/s²，XYZ保持")
    parser.add_argument("--ten-degree-orientation-acceptance", action="store_true",
                        help="十段累计≤10°纯姿态真机验收：每段≤1°、5°/s、10°/s²，XYZ保持")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", ""))
    parser.add_argument("--speed-mm-s", type=float, default=10)
    parser.add_argument("--radius-mm", type=float, default=100)
    parser.add_argument("--acceleration-mm-s2", type=float, default=50)
    parser.add_argument("--session-seconds", type=float, default=1800)
    parser.add_argument("--limits-file", type=Path, default=Path(os.environ.get(
        "QUEST_JAKA_LIMITS_FILE", "__SET_QUEST_JAKA_LIMITS_FILE_IN_LOCAL_CONFIG__")))
    args = parser.parse_args(argv)
    if not (args.live or args.shadow or args.comm_check or args.rotation_shadow
            or args.one_degree_orientation_acceptance
            or args.three_degree_orientation_acceptance
            or args.ten_degree_orientation_acceptance or args.bounded_six_dof
            or args.expanded_six_dof or args.production_six_dof):
        print("默认不连接。通过②界面选择只读预览或真机短段跟随。")
        return 0
    if args.live and not args.ui_heartbeat:
        parser.error("真机模式必须由提供持续心跳的②界面启动")
    if sum((args.one_segment_acceptance, args.two_segment_acceptance,
            args.five_segment_acceptance, args.ten_segment_acceptance,
            args.twenty_segment_acceptance, args.one_degree_orientation_acceptance,
            args.three_degree_orientation_acceptance,
            args.ten_degree_orientation_acceptance)) > 1:
        parser.error("分阶段验收模式不能同时启动")
    # 0 表示只读影子/通信检查；非零值是本次最多可下发的短段条数。
    orientation_acceptance = bool(args.one_degree_orientation_acceptance
                                  or args.three_degree_orientation_acceptance
                                  or args.ten_degree_orientation_acceptance)
    if args.ten_degree_orientation_acceptance:
        acceptance_count = 10
    elif args.three_degree_orientation_acceptance:
        acceptance_count = 3
    elif args.one_degree_orientation_acceptance:
        acceptance_count = 1
    elif args.one_segment_acceptance:
        acceptance_count = 1
    elif args.two_segment_acceptance:
        acceptance_count = 2
    elif args.five_segment_acceptance:
        acceptance_count = 5
    elif args.ten_segment_acceptance:
        acceptance_count = 10
    elif args.twenty_segment_acceptance:
        acceptance_count = 20
    else:
        acceptance_count = 0
    if acceptance_count and not (args.live and args.ui_heartbeat and args.single_owner_display):
        parser.error("真机验收必须由②界面以单连接实测反馈模式启动")
    if args.bounded_continuous and acceptance_count:
        parser.error("受限连续模式不能与分阶段验收同时启动")
    six_dof_modes = sum((args.bounded_six_dof, args.expanded_six_dof,
                         args.production_six_dof))
    if six_dof_modes and (acceptance_count or args.bounded_continuous
                          or six_dof_modes > 1):
        parser.error("受限连续六维不能与其它真机档位同时启动")
    if args.bounded_continuous and not (args.live and args.ui_heartbeat and args.single_owner_display):
        parser.error("受限连续真机跟随必须由②界面以单连接实测反馈模式启动")
    if six_dof_modes and not (
            args.live and args.ui_heartbeat and args.single_owner_display):
        parser.error("受限连续六维必须由②界面以单连接实测反馈模式启动")
    if args.live and not (acceptance_count or args.bounded_continuous
                          or args.bounded_six_dof or args.expanded_six_dof
                          or args.production_six_dof):
        parser.error("真机模式缺少已开放的受限档位")
    if args.rotation_shadow and not (args.ui_heartbeat and args.single_owner_display):
        parser.error("姿态影子必须由②界面以单连接实测反馈模式启动")
    if args.ui_heartbeat and args.ui_protocol != 2:
        parser.error("界面协议已升级，请关闭旧②窗口并重新双击②；不要在旧窗口启动新版运行器")
    # 分阶段验收参数由档位固定；受限连续模式才读取界面参数，并再次执行硬上限检查。
    if orientation_acceptance:
        settings = Settings(radius_mm=20, speed_mm_s=5, acceleration_mm_s2=10,
                            segment_mm=5, deadband_mm=3)
    elif args.one_segment_acceptance:
        settings = Settings(radius_mm=20, speed_mm_s=5, acceleration_mm_s2=10,
                            segment_mm=5, deadband_mm=3)
    elif args.two_segment_acceptance:
        settings = Settings(radius_mm=30, speed_mm_s=10, acceleration_mm_s2=20,
                            segment_mm=10, deadband_mm=3)
    elif args.five_segment_acceptance:
        settings = Settings(radius_mm=60, speed_mm_s=15, acceleration_mm_s2=30,
                            segment_mm=10, deadband_mm=3)
    elif args.ten_segment_acceptance:
        settings = Settings(radius_mm=100, speed_mm_s=20, acceleration_mm_s2=40,
                            segment_mm=10, deadband_mm=3)
    elif args.twenty_segment_acceptance:
        settings = Settings(radius_mm=200, speed_mm_s=30, acceleration_mm_s2=60,
                            segment_mm=10, deadband_mm=3)
    elif args.production_six_dof:
        if not 200 <= args.radius_mm <= 1000:
            parser.error("正式六维活动半径必须为200—1000mm")
        if not 5 <= args.speed_mm_s <= 100:
            parser.error("正式六维TCP速度必须为5—100mm/s")
        if not 10 <= args.acceleration_mm_s2 <= 400:
            parser.error("正式六维TCP加速度必须为10—400mm/s²")
        if not 10 <= args.rotation_radius_deg <= 30:
            parser.error("正式六维姿态范围必须为10—30°")
        if not 1 <= args.orientation_speed_deg_s <= 20:
            parser.error("正式六维姿态速度必须为1—20°/s")
        settings = Settings(radius_mm=args.radius_mm,speed_mm_s=args.speed_mm_s,
                            acceleration_mm_s2=args.acceleration_mm_s2,
                            segment_mm=20,deadband_mm=3)
    elif args.expanded_six_dof:
        settings = Settings(radius_mm=500,speed_mm_s=50,acceleration_mm_s2=100,
                            segment_mm=20,deadband_mm=3)
    elif args.bounded_six_dof:
        settings = Settings(radius_mm=200,speed_mm_s=30,acceleration_mm_s2=60,
                            segment_mm=10,deadband_mm=3)
    elif args.bounded_continuous:
        # 连续模式只采用界面启动时锁定的参数；即使直接调用CLI也不能越过现场已验收上限。
        if not 20 <= args.radius_mm <= 200:
            parser.error("受限连续模式活动半径必须为20—200mm")
        if not 5 <= args.speed_mm_s <= 30:
            parser.error("受限连续模式速度必须为5—30mm/s")
        if not 10 <= args.acceleration_mm_s2 <= 60:
            parser.error("受限连续模式加速度必须为10—60mm/s²")
        settings = Settings(radius_mm=args.radius_mm, speed_mm_s=args.speed_mm_s,
                            acceleration_mm_s2=args.acceleration_mm_s2,
                            segment_mm=10, deadband_mm=3)
    else:
        settings = Settings(radius_mm=args.radius_mm, speed_mm_s=args.speed_mm_s,
                            acceleration_mm_s2=args.acceleration_mm_s2)
    if acceptance_count:
        if args.ten_degree_orientation_acceptance:
            maximum_seconds = 90
        elif args.three_degree_orientation_acceptance:
            maximum_seconds = 45
        elif args.one_degree_orientation_acceptance:
            maximum_seconds = 30
        elif args.twenty_segment_acceptance:
            maximum_seconds = 180
        elif args.ten_segment_acceptance:
            maximum_seconds = 120
        elif args.five_segment_acceptance:
            maximum_seconds = 90
        elif args.two_segment_acceptance:
            maximum_seconds = 45
        else:
            maximum_seconds = 30
        args.session_seconds = min(args.session_seconds, maximum_seconds)
    elif (args.bounded_continuous or args.bounded_six_dof or args.expanded_six_dof
          or args.production_six_dof):
        args.session_seconds = min(args.session_seconds, 600)
    elif args.rotation_shadow:
        args.session_seconds = min(args.session_seconds, 30)
    if not 1 <= args.session_seconds <= 1800:
        parser.error("会话时长必须为1—1800秒")
    host = validate_host(args.host)
    limits, limits_hash = read_limits(args.limits_file)
    mapping = json.loads((ROOT / "Python/config/jaka_jog.json").read_text(encoding="utf-8"))["quest_vr"]["direction_mapping"]
    folder = SESSION_LOG_ROOT
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / datetime.now().strftime("session_%Y%m%d_%H%M%S_%f.jsonl")
    permit = UiPermit() if args.ui_heartbeat else None
    robot = receiver = overlay = follower = feedback = lease = None
    logged = False
    failure = None
    stopped = True
    latest = None
    last_status = ""
    display_binding = None
    sdk_trace = TraceFile(path.with_suffix(".sdk.jsonl"))
    with path.open("w", encoding="utf-8", buffering=1) as log:
        def emit(**event):
            """事件先写本地 JSONL，再按需显示给 GUI/转发目标标记给 Unity。

            `target` 和 `display_binding` 是待去的位置；真正的机械臂姿态只取
            `SingleOwnerFeedback` 的实测关节/TCP，不能混成同一条反馈。
            """
            nonlocal last_status, display_binding
            event["time_ns"] = time.time_ns()
            log.write(json.dumps(event, ensure_ascii=False) + "\n")
            status = event.get("state", "")
            if status not in ("target", "display_binding") and status != last_status:
                print(json.dumps(event, ensure_ascii=False), flush=True)
                last_status = status
            if status == "display_binding":
                display_binding = event.get("binding")
            if status == "finished":
                display_binding = None
            if overlay is not None and (status in ("display_binding", "target", "finished")):
                # 直接交给Unity；不绕经①的SDK读取线程。重复绑定不重新捕获手柄。
                packet = {"schema": "quest_jaka_target_binding.v1", "binding": display_binding,
                          "sent_time_ns": time.time_ns()}
                try:
                    overlay.sendto(json.dumps(packet).encode("utf-8"), ("127.0.0.1", 5006))
                except OSError:
                    pass

        emit(state="configuration", settings=asdict(settings), live=args.live,
             one_segment_acceptance=args.one_segment_acceptance,
             two_segment_acceptance=args.two_segment_acceptance,
             five_segment_acceptance=args.five_segment_acceptance,
             ten_segment_acceptance=args.ten_segment_acceptance,
              twenty_segment_acceptance=args.twenty_segment_acceptance,
              one_degree_orientation_acceptance=args.one_degree_orientation_acceptance,
              three_degree_orientation_acceptance=args.three_degree_orientation_acceptance,
              ten_degree_orientation_acceptance=args.ten_degree_orientation_acceptance,
              bounded_continuous=args.bounded_continuous,
              bounded_six_dof=args.bounded_six_dof,
              expanded_six_dof=args.expanded_six_dof,
              production_six_dof=args.production_six_dof,
              rotation_radius_deg=args.rotation_radius_deg,
              orientation_speed_deg_s=args.orientation_speed_deg_s,
             rotation_shadow=args.rotation_shadow,
             communication_check=args.comm_check,
             report=str(path), sdk_trace=str(path.with_suffix(".sdk.jsonl")),
             limits_file=str(args.limits_file), limits_sha256=limits_hash, limits_deg=limits)
        # 唯一持有 SDK 的会话边界；任何提前返回都仍会进入 finally 清理。
        try:
            # 新①在单独运行时负责只读实测显示。②接管前先请求它注销并释放互斥锁；
            # 若仍是旧①，下面的端口检测会拒绝双连接。
            if not request_readonly_handoff():
                reject_legacy_bridge()
            lease = SdkOwnerLease()
            sdk = load_sdk(SDK_DIRECTORY)
            # 每次原生调用的起止独立落盘；GUI只显示异常，不把高频诊断刷进文本框。
            robot = TracedSdk(sdk.RC(host), sdk_trace,
                              progress=lambda event: print(json.dumps(event, ensure_ascii=False), flush=True))
            checked(robot.login(), "login")
            logged = True
            # 只读通信检查单独分支：不创建 SampledFollower，也不会进入 dispatch。
            if args.comm_check:
                # 只读压力检查可以分别验证单/双连接，不预先把失败归因于多连接。
                # 不上电、不使能、不改滤波；SDK诊断记录每条只读调用。
                if args.single_owner_display:
                    feedback = SingleOwnerFeedback(robot)
                started = time.perf_counter()
                deadline, health_due, samples = started, started, 0
                emit(state="comm_check", message="只读通信检查进行中，不需要握Grip，不发送运动")
                while time.perf_counter()-started < args.session_seconds:
                    if permit is not None and not permit.valid(): break
                    checked(robot.get_actual_joint_position(), "只读关节")
                    checked(robot.get_actual_tcp_position(), "只读TCP")
                    if time.perf_counter() >= health_due:
                        for method in ("get_robot_status_simple", "get_tool_id", "get_user_frame_id",
                                       "is_in_estop", "is_in_collision", "is_on_limit", "is_in_servomove"):
                            checked(getattr(robot, method)(), method)
                        health_due = time.perf_counter()+.05
                    if feedback is not None:
                        feedback.tick()
                    samples += 1
                    deadline = max(deadline+1/60, time.perf_counter())
                    time.sleep(max(0,deadline-time.perf_counter()))
                elapsed = time.perf_counter()-started
                emit(state="comm_result", message="只读检查结束（不是运动放行）", samples=samples,
                     seconds=elapsed, measured_hz=samples/max(elapsed,.001), movement_commands_sent=0)
                return 0
            if (orientation_acceptance or args.bounded_six_dof or args.expanded_six_dof
                    or args.production_six_dof) and not hasattr(robot, "linear_move_extend_ori"):
                raise RuntimeError("当前SDK缺少linear_move_extend_ori，拒绝姿态真机验收")
            if not (args.rotation_shadow or orientation_acceptance or args.bounded_six_dof
                    or args.expanded_six_dof or args.production_six_dof) and not hasattr(robot, "linear_move_extend"):
                raise RuntimeError("当前SDK缺少linear_move_extend，拒绝退回隐式加速度")
            if checked(robot.get_user_frame_id(), "用户坐标系") != 0:
                raise RuntimeError("要求用户坐标系0，与基坐标映射一致")
            receiver = QuestUdpReceiver("127.0.0.1", 5005)
            overlay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if args.rotation_shadow:
                # 与运动会话相同的SDK独占/心跳/实测显示架构，但本分支没有任何运动调用。
                check = RotationShadow(robot, mapping, emit)
                feedback = SingleOwnerFeedback(robot)
                started = time.monotonic()
                health_due = started
                emit(state="rotation_shadow_start",
                     message="30秒姿态只读影子；黄色轴随手柄旋转，运动命令恒为0")
                while time.monotonic()-started < args.session_seconds:
                    if permit is not None and not permit.valid():
                        break
                    if receiver.error:
                        raise RuntimeError(f"Quest接收失败：{receiver.error}")
                    frame = receiver.latest()
                    now = time.monotonic()
                    if frame is not None:
                        latest = frame
                    check.ready_heading(latest, now)
                    check.process(latest, now)
                    if now >= health_due:
                        # 只读检查仍核对整组状态；不调用任何运动、上电或使能接口。
                        check_health(robot)
                        health_due = time.monotonic()+.2
                    feedback.tick()
                    if latest and latest.button_b:
                        break
                    time.sleep(.01)
                emit(state="rotation_shadow_complete",
                     message="姿态只读影子结束；仍未开放姿态真机运动",
                     solved_frames=check.solved_frames,
                     blocked_frames=check.blocked_frames,
                     movement_commands_sent=0)
                return 0
            if args.production_six_dof:
                follower = BoundedPoseFollower(
                    robot,mapping,settings,limits,emit=emit,
                    rotation_radius_deg=args.rotation_radius_deg,
                    orientation_speed_deg_s=args.orientation_speed_deg_s,
                    orientation_acceleration_deg_s2=min(80.0, 4.0*args.orientation_speed_deg_s),
                    max_orientation_step_deg=2.0)
            elif args.expanded_six_dof:
                follower = BoundedPoseFollower(
                    robot,mapping,settings,limits,emit=emit,
                    rotation_radius_deg=30.0,orientation_speed_deg_s=10.0,
                    orientation_acceleration_deg_s2=20.0,max_orientation_step_deg=2.0)
            elif args.bounded_six_dof:
                follower = BoundedPoseFollower(robot, mapping, settings, limits, emit=emit)
            elif args.ten_degree_orientation_acceptance:
                follower = OrientationAcceptance(
                    robot, mapping, limits, emit=emit, command_limit=10,
                    max_total_deg=10.0, orientation_speed_deg_s=5.0,
                    orientation_acceleration_deg_s2=10.0)
            elif args.three_degree_orientation_acceptance:
                follower = OrientationAcceptance(
                    robot, mapping, limits, emit=emit, command_limit=3,
                    max_total_deg=3.0, orientation_speed_deg_s=2.0,
                    orientation_acceleration_deg_s2=4.0)
            elif args.one_degree_orientation_acceptance:
                follower = OrientationAcceptance(robot, mapping, limits, emit=emit)
            else:
                follower = SampledFollower(
                    robot, mapping, settings, limits, emit=emit,
                    command_limit=acceptance_count or None,
                    path_limit_mm=(acceptance_count * settings.segment_mm) if acceptance_count else None,
                )
            follower.initialize()
            if acceptance_count:
                if orientation_acceptance:
                    emit(state="orientation_acceptance_start",
                         measured_joints=follower.initial_joints,
                         measured_tcp=follower.center,
                         command_limit=acceptance_count,
                         total_limit_deg=follower.max_total_deg,
                         per_command_limit_deg=1.0)
                else:
                    emit(state="acceptance_start", measured_joints=follower.initial_joints,
                         measured_tcp=follower.center, command_limit=acceptance_count,
                         path_limit_mm=acceptance_count * settings.segment_mm)
            if args.single_owner_display:
                feedback = SingleOwnerFeedback(robot)
                feedback.tick()
                emit(state="display", message="②独占SDK连接；Unity显示实测关节反馈，不显示伪造运动")
            started = time.monotonic()
            # 真机/影子主循环：每轮先处理最新输入，再判断能否准备或下发短段。
            while time.monotonic() - started < args.session_seconds:
                if permit is not None and not permit.valid():
                    emit(state="stopping", message="界面停止或心跳中断")
                    break
                if receiver.error:
                    raise RuntimeError(f"Quest接收失败：{receiver.error}")
                frame = receiver.latest()
                if frame is not None:
                    latest = frame
                follower.ready_heading(latest)
                acceptance_waiting = bool(acceptance_count and follower.commands >= acceptance_count)
                follower.tick(latest, allow_plan=not acceptance_waiting)
                # IK期间输入仍由UDP线程更新；发命令前重新检查最新Grip/追踪/心跳。
                frame = receiver.latest()
                if frame is not None:
                    latest = frame
                if acceptance_waiting:
                    if not follower.active:
                        if follower.completed_segments == acceptance_count:
                            end = follower.last_completed_measurement
                            if orientation_acceptance:
                                emit(state="orientation_acceptance_complete",
                                     message=("十段累计≤10°姿态命令均已实测到位；不再下发运动"
                                              if args.ten_degree_orientation_acceptance else
                                              "三段累计≤3°姿态命令均已实测到位；不再下发运动"
                                              if args.three_degree_orientation_acceptance else
                                              "唯一≤1°姿态命令已实测到位；不再下发运动"),
                                     movement_commands_sent=follower.commands,
                                     measured_joints=end[0], measured_tcp=end[1],
                                     commanded_orientation_deg=follower.commanded_orientation_deg)
                            else:
                                emit(state="acceptance_complete", message="受限短段均已实测到位；不再下发运动",
                                     movement_commands_sent=follower.commands,
                                     measured_joints=end[0], measured_tcp=end[1],
                                     measured_displacement_mm=math.dist(follower.center[:3], end[1][:3]),
                                     measured_path_mm=follower.measured_path_mm,
                                     commanded_path_mm=follower.commanded_path_mm)
                        else:
                            failure = "受限短段未全部到位或被中止；不得自动重试"
                            emit(state="acceptance_incomplete", message=failure)
                        break
                elif args.live:
                    before = follower.commands
                    def refresh_permission():
                        nonlocal latest
                        fresh = receiver.latest()
                        if fresh is not None:
                            latest = fresh
                        return latest, bool(permit and permit.valid() and not receiver.error)
                    t0 = time.perf_counter()
                    follower.dispatch(latest, permit=permit.valid() if permit else False,
                                      refresh=refresh_permission)
                    if follower.commands != before:
                        emit(state="command", target=follower.active_target,
                             checked_joint_path=follower.path_solutions,
                             dispatch_checks_and_call_ms=(time.perf_counter()-t0)*1000)
                elif getattr(follower, "pending", None) is not None:
                    emit(state="shadow", target=follower.pending[0], message="只读短段检查通过；未运动")
                    follower.pending = None
                if feedback is not None:
                    feedback.tick()
                if latest and latest.button_b:
                    break
                time.sleep(.01)
            if acceptance_count and follower.completed_segments != acceptance_count and failure is None:
                failure = "受限验收在全部到位前结束；不得自动重试"
                emit(state="acceptance_incomplete", message=failure)
        except Exception as error:
            failure = str(error)
            emit(state="fault", message=failure)
        finally:
            if follower is not None:
                try:
                    stopped = follower.shutdown()
                except Exception as error:
                    stopped = False
                    emit(state="stop_unconfirmed", message=str(error))
            if receiver is not None:
                receiver.close()
            emit(state="finished", stop_confirmed=stopped, failure=failure,
                 movement_commands_sent=follower.commands if follower else 0, target=None)
            if overlay is not None:
                overlay.close()
            if feedback is not None:
                feedback.close()
            if logged:
                try:
                    checked(robot.logout(), "logout")
                except Exception as error:
                    emit(state="logout_warning", message=str(error))
            if lease is not None:
                lease.close()
            sdk_trace.close()
    print(f"报告：{path}", flush=True)
    return 0 if failure is None and stopped else 2


if __name__ == "__main__":
    raise SystemExit(main())
