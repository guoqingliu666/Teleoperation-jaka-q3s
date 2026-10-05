"""只观测候选规划，不修改指令；TCP变换以机器人基座、米为单位。

孪生UDP与审计日志复用同一份数据。用户配置工具数学模型只用于影子显示，
不以模型FK冒充实机TCP，也不把手柄原始追踪坐标直接当机器人坐标。
"""
import numpy as np
from .global_kinematics import rotation_log


def shadow_tracking(planner, feedback_transform, frame, now, model_sha256):
    tcp=np.asarray(feedback_transform,dtype=float)
    if tcp.shape!=(4,4) or not np.isfinite(tcp).all():
        raise ValueError('invalid shadow feedback transform')
    fresh=bool(frame and frame.get('tracking_valid') and
               0<=now-frame['stamp']<=planner.cfg['safety']['input_timeout_s'])
    active=bool(fresh and planner.held and planner.phase=='FOLLOW' and planner.goal is not None)
    goal=planner.goal if active else None
    sigma=planner.servo.sigma if active else None
    scale=planner.servo.velocity_scale if active else None
    warnings=[]
    if sigma is not None and sigma<.02: warnings.append('NEAR_SINGULARITY')
    if scale is not None and scale<.999: warnings.append('JOINT_SPEED_LIMIT')
    if planner.servo.boundary_braking: warnings.append('JOINT_BOUNDARY_BRAKE')
    if not active: warnings.append('TARGET_UNBOUND')
    return dict(schema=1,frame='robot_base',transform_layout='row_major_4x4_m',
        feedback_source='simulated_fk',tool_model=getattr(planner.model,'tool_frame_id','frozen_old_tool1_not_current_controller'),
        tool_config_sha256=getattr(planner.model,'tool_provenance',{}).get('config_sha256'),
        tcp2_T=(tcp @ np.array([[1.,0,0,0],[0,1.,0,0],[0,0,1.,-.2],[0,0,0,1.]])).reshape(-1).tolist()
            if hasattr(planner.model,'tcp2_model') else None,
        model_sha256=model_sha256,feedback_T=tcp.reshape(-1).tolist(),
        target_T=None if goal is None else goal.reshape(-1).tolist(),target_valid=active,
        sample_clock_s=float(now),input_age_ms=None if frame is None else (now-frame['stamp'])*1000,
        position_error_mm=None if goal is None else float(np.linalg.norm(goal[:3,3]-tcp[:3,3])*1000),
        orientation_error_deg=None if goal is None else float(np.rad2deg(np.linalg.norm(rotation_log(goal[:3,:3]@tcp[:3,:3].T)))),
        solver_valid=active,sigma_min=sigma,joint_velocity_scale=scale,
        feedback_scale=float(planner.servo.feedback_scale),
        requested_twist=planner.requested_twist.tolist(),
        filtered_twist=planner.servo.filtered_twist.tolist() if active else [0.]*6,
        unconstrained_joint_velocity=planner.servo.raw_joint_velocity.tolist() if active else [0.]*6,
        otg_target_velocity=list(planner.servo.input.target_velocity),
        warnings=warnings,physical_feedback=False)
