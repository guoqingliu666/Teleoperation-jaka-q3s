"""Apply a cabinet TCP to the calibrated baseline model without changing joints."""
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation

BASELINE_TCP = [0., 0., 200., 0., 0., 0.]

def tool_values(payload):
    tool = payload.get('tool_id', 1)
    tcp = payload.get('tcp_mm_rad', BASELINE_TCP)
    if type(tool) is not int or not 0 <= tool <= 10:
        raise ValueError('工具号必须为 0（法兰）或 TCP1–TCP10')
    if not isinstance(tcp, list) or len(tcp) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in tcp):
        raise ValueError('TCP 定义必须是六个有限数值，单位 mm / rad')
    return tool, list(tcp)

def pose_matrix(values):
    matrix = np.eye(4); matrix[:3,3] = np.asarray(values[:3])/1000
    matrix[:3,:3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    return matrix

def apply_tool(robot, metadata, payload):
    """Inputs are BASELINE model/metadata. Return a consistent, independent pair."""
    _, tcp = tool_values(payload)
    robot = copy.deepcopy(robot); metadata = copy.deepcopy(metadata)
    change = np.linalg.inv(pose_matrix(BASELINE_TCP)) @ pose_matrix(tcp)
    element = robot.find("joint[@name='tool_mount']/origin")
    values = [float(v)*1000 for v in element.get('xyz').split()] + [float(v) for v in element.get('rpy').split()]
    mount = pose_matrix(values) @ change
    element.set('xyz', ' '.join(format(v, '.16g') for v in mount[:3,3]))
    element.set('rpy', ' '.join(format(v, '.16g') for v in Rotation.from_matrix(mount[:3,:3]).as_euler('xyz')))
    metadata['tcp_home'] = (np.array(metadata['tcp_home']) @ change).tolist()
    return robot, metadata
