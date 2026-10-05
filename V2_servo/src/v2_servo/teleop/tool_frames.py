"""从历史官方 FK 去掉旧工具，再沿 TCP2 局部 +Z 加 200 mm。

只构造影子模型；历史报告不改写，新工具不继承历史实测验收身份。
TCP2 对应恢复的法兰参考系；传感器实体注册仍需现场核对。
"""
import hashlib
import json
import numpy as np
from .global_kinematics import SerialPoE, pose_matrix


def configured_shadow_tool(report, config_path):
    raw = config_path.read_bytes()
    config = json.loads(raw)
    if config.get('scope') != 'user_configured_shadow_tool_not_live_calibration':
        raise ValueError('invalid shadow tool scope')
    offset = pose_matrix(config['tcp1_from_tcp2_mm_rad'])
    if not np.allclose(offset, pose_matrix([0, 0, 200, 0, 0, 0]), atol=1e-12):
        raise ValueError('TCP1 configuration differs from confirmed +Z 200 mm')
    original = SerialPoE(report['model'])
    tcp2_home = original.home @ np.linalg.inv(pose_matrix(report['tcp_parameters']))
    tcp2_record = dict(report['model'], T_reference_m=tcp2_home.tolist())
    model = SerialPoE(dict(report['model'], T_reference_m=(tcp2_home @ offset).tolist()))
    model.tcp2_model = SerialPoE(tcp2_record)
    model.tool_frame_id = 'TCP1_plus200mm_localZ_user_configured_shadow'
    model.tool_provenance = dict(config, config_sha256=hashlib.sha256(raw).hexdigest(),
        old_tcp_parameters_mm_rad=report['tcp_parameters'],
        derivation='T_base_TCP1 = T_base_oldTCP @ inverse(T_flange_oldTCP) @ TransZ(0.2m)')
    return model
