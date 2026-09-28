"""带时间依据的关节路径诊断；不规划/发送机器人运动。

有限采样只能给出区间平均速度、差分加速度，不能证明采样间峰值受限。
尤其不能把这个报告当成厂商圆滑过渡的加速度证书。所有角度保持原始连续
关节值，不取模；每个 Grip/失效边界重置，不跨边界差分。
"""
from __future__ import annotations

from dataclasses import dataclass
import configparser
import hashlib
import math
from pathlib import Path

from .sampled_follow import six


@dataclass(frozen=True)
class JointBudgets:
    speed_deg_s: tuple
    acceleration_deg_s2: tuple
    source: str

    def __post_init__(self):
        for values in (self.speed_deg_s,self.acceleration_deg_s2):
            if len(values) != 6 or any(not math.isfinite(x) or x <= 0 for x in values):
                raise ValueError("六轴速度/加速度预算必须为有限正数")
        if not self.source:
            raise ValueError("关节预算必须标明来源")


def inspect_exported_joint_settings(path):
    """仅记录导出快照的原始值及哈希；不擅自把单位未知的数值用作运行限制。

    ini 中无单位、导出时间或当前生效证明。调用者需单独确认单位和运行预算，
    因此本函数故意不返回 JointBudgets，也没有补缺省最大值。
    """
    data=Path(path).read_bytes()
    config=configparser.ConfigParser(strict=True)
    config.read_string(data.decode("utf-8-sig"))
    entries=[]
    for i in range(6):
        section=config[f"JOINT_{i}"]
        item={key:float(section[key]) for key in
              ("JOINT_MIN_LIMIT","JOINT_MAX_LIMIT","JOINT_VEL_LIMIT","JOINT_ACC_LIMIT")}
        if (any(not math.isfinite(v) for v in item.values())
                or item["JOINT_MIN_LIMIT"] >= item["JOINT_MAX_LIMIT"]
                or min(item["JOINT_VEL_LIMIT"],item["JOINT_ACC_LIMIT"]) <= 0):
            raise ValueError(f"JOINT_{i}配置值无效")
        entries.append(item)
    return {"source":str(Path(path).resolve()),"sha256":hashlib.sha256(data).hexdigest(),
            "raw_joint_settings":entries,"units_confirmed":False,
            "current_controller_values_verified":False,"applied_to_robot":False}


class JointTimingAudit:
    """跨候选段保留上一段速度，才能发现段间反向造成的加速度需求。

    t 是要求评估的计划时间，不是SDK耗时。若没有厂商执行时间，报告必须标记
    为手柄时间尺度，不能说是真机速度。本类既不改解，也不生成其它逆解。
    """
    def __init__(self, *, time_basis, budgets=None):
        if not time_basis:
            raise ValueError("必须声明时间依据")
        self.time_basis,self.budgets=time_basis,budgets
        self.reset()

    def reset(self):
        self.previous=self.previous_velocity=self.previous_midpoint=None
        self.speed=[0.]*6
        self.acceleration=[0.]*6
        self.count=self.acceleration_count=0

    def push(self,t,joints):
        q=six(joints)
        if not math.isfinite(t):
            raise ValueError("时间不是有限数")
        if self.previous is not None:
            before_t,before_q=self.previous
            dt=t-before_t
            if dt <= 1e-9:
                raise ValueError("差分时间必须严格递增且大于1纳秒")
            velocity=tuple(math.degrees(b-a)/dt for a,b in zip(before_q,q))
            midpoint=before_t+dt/2
            if not all(math.isfinite(x) for x in velocity):
                raise ValueError("速度差分溢出")
            if self.previous_velocity is not None:
                acc=tuple((v-u)/(midpoint-self.previous_midpoint)
                          for u,v in zip(self.previous_velocity,velocity))
                if not all(math.isfinite(x) for x in acc):
                    raise ValueError("加速度差分溢出")
                self.acceleration=[max(a,abs(b)) for a,b in zip(self.acceleration,acc)]
                self.acceleration_count+=1
            self.speed=[max(a,abs(b)) for a,b in zip(self.speed,velocity)]
            self.previous_velocity,self.previous_midpoint=velocity,midpoint
        self.previous=(t,q)
        self.count+=1

    def report(self):
        scale=None
        if self.budgets is not None and self.acceleration_count:
            # 统一时间伸缩下离散速度按1/k、离散加速度按1/k²变化。
            # 这是离散诊断必要的缩放，不是连续运动的充分安全约束。
            scale=max(1.,*(v/b for v,b in zip(self.speed,self.budgets.speed_deg_s)),
                      *(math.sqrt(a/b) for a,b in zip(self.acceleration,self.budgets.acceleration_deg_s2)))
        return {"time_basis":self.time_basis,"samples":self.count,
                "acceleration_estimates":self.acceleration_count,
                "max_interval_average_speed_deg_s":self.speed.copy(),
                "max_difference_acceleration_deg_s2":self.acceleration.copy(),
                "sampled_uniform_time_scale":scale,
                "budget_source":self.budgets.source if self.budgets else None,
                "continuous_peak_bounds_proven":False,"applied_to_robot":False}
