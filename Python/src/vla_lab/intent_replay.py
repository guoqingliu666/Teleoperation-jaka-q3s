"""真实手柄日志 + 明确标记的逆解替身，用于调度离线压力回放。

替身仅提供连续的关节映射及可控耗时，不代表 JAKA 运动学，不验证可达性、
奇异点或碰撞。程序不加载 SDK。将规划结果按名义段时间消费以模拟背压；
不能把这个虚拟消费者当成真实控制柜圆滑运动。
"""
import argparse
import json
import math
from pathlib import Path

from .intent_horizon import IntentHorizon
from .trajectory_horizon import TimeHorizon
from .trajectory_reference import PoseSample


class ReplayClock:
    def __init__(self):
        self.now = 0.
    def __call__(self):
        return self.now


def replay(events, *, legacy=False, consumer_scale=1.):
    clock = ReplayClock()
    counters = dict(input_samples=0, candidates=0, blocked_windows=0,
                    maximum_buffer_points=0, maximum_pending_nominal_s=0.,
                    limited_frames=0, planning_slice_plans=0, inverse_calls=0)
    windows, current = [], None
    for e in events:
        if e.get("state") == "trajectory_preview_anchor":
            current = {"anchor": e, "samples": []}
            windows.append(current)
        if e.get("state") == "trajectory_raw_sample" and current is not None:
            current["samples"].append(PoseSample(e["t"],tuple(e["xyz"]),tuple(e["q"])))
    failures = []
    for window in windows:
        samples = window["samples"]
        if not samples:
            continue
        first = samples[0]
        clock.now = first.t
        anchor = PoseSample.from_tcp(first.t,window["anchor"]["anchor_tcp"])
        origin_tcp = anchor.tcp()
        joints = tuple(window["anchor"]["measured_joints"])
        def inverse(seed, tcp):
            counters["inverse_calls"] += 1
            # 1.11ms常规耗时与20.74ms周期尖峰取自现场观测的均值/最大值；
            # 它是压力模型，不伪称重放了每次真实 SDK 调用的耗时。
            clock.now += .02074 if counters["inverse_calls"] % 50 == 0 else .00111
            q = list(joints)
            for axis in range(3):
                q[axis] += (tcp[axis]-origin_tcp[axis])*.001
            for axis in range(3,6):
                value = joints[axis] + tcp[axis]-origin_tcp[axis]
                q[axis] = value+round((seed[axis]-value)/(2*math.pi))*2*math.pi
            return 0,tuple(q)
        factory = TimeHorizon if legacy else IntentHorizon
        h = factory(inverse, ((-10000.,10000.),)*6, clock=clock, adaptive=True,
                    **({} if legacy else {"rotation_radius_deg":180.}))
        h.reset(anchor,joints)
        index, next_slot = 1, clock.now
        end = samples[-1].t + 2.
        last_heartbeat = first.t
        try:
            while clock.now < end:
                while index < len(samples) and samples[index].t <= clock.now:
                    p = samples[index]
                    h.push(p,p.t)
                    last_heartbeat = p.t
                    counters["input_samples"] += 1
                    index += 1
                if index == len(samples) and clock.now-last_heartbeat >= .02:
                    p = PoseSample(clock.now,samples[-1].xyz,samples[-1].q)
                    h.push(p,clock.now)
                    last_heartbeat = clock.now
                h.advance(permit=True)
                counters["maximum_buffer_points"] = max(counters["maximum_buffer_points"],len(h.samples))
                counters["maximum_pending_nominal_s"] = max(counters["maximum_pending_nominal_s"],
                    h.pending_seconds() if not legacy else 0.)
                if clock.now >= next_slot:
                    plan = h.take_when_slot_open(expected_epoch=h.epoch)
                    if plan is not None:
                        counters["candidates"] += 1
                        counters["planning_slice_plans"] += plan.completion_reason in ("planning_slice","backlog_slice")
                        next_slot = clock.now + max(.04,plan.duration_lower_bound_s)*consumer_scale
                clock.now += .002
        except (RuntimeError,ValueError) as error:
            counters["blocked_windows"] += 1
            failures.append(str(error))
        counters["limited_frames"] += getattr(h,"limited_frames",0)
    return dict(**counters, failures=failures, windows=len(windows),
                consumer_scale=consumer_scale, legacy=legacy,
                sdk_used=False, movement_commands_sent=0,
                inverse_model="synthetic_continuous_mapping_not_robot_kinematics",
                physical_acceptance=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log",type=Path)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    events=[json.loads(line) for line in args.log.read_text(encoding="utf-8").splitlines()]
    results={"source":str(args.log),"runs":[replay(events,legacy=old,consumer_scale=scale)
             for scale in (1.,3.) for old in (True,False)]}
    text=json.dumps(results,ensure_ascii=False,indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(text+"\n",encoding="utf-8")
    print(text)

if __name__ == "__main__":
    main()
