"""Quest UDP -> 有原始接收时间的末端目标。只连接输入，不拥有 SDK。"""
import json
import socket
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from .input import Clutch


class QuestBridge(Node):
    def __init__(self):
        super().__init__('quest_bridge')
        self.declare_parameter('bind', '0.0.0.0'); self.declare_parameter('port', 5006)
        self.declare_parameter('basis', [0.,0.,1.,-1.,0.,0.,0.,1.,0.])
        self.clutch = Clutch(np.array(self.get_parameter('basis').value).reshape(3,3))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.get_parameter('bind').value, self.get_parameter('port').value))
        self.socket.setblocking(False)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(PoseStamped, '/v3/hand_target', 1)
        self.state = self.create_publisher(String, '/v3/input_state', 1)
        self.feedback_at = -float('inf')
        self.create_subscription(JointState, '/joint_states', self.feedback, 1)
        self.create_timer(.004, self.receive)

    def feedback(self, msg):
        if all(f'joint_{i}' in msg.name for i in range(1,7)) and np.isfinite(msg.position).all():
            age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec))/1e9
            if 0 <= age <= .024: self.feedback_at = time.monotonic()

    def receive(self):
        now = time.monotonic(); self.clutch.expire(now)
        # 有界排空，处理所有追踪/停止边沿；过量排队直接失效，不能把老包续成新输入。
        for index in range(65):
            try: raw, source = self.socket.recvfrom(8192)
            except BlockingIOError: break
            if index == 64:
                self.clutch.invalidate('INPUT_BACKLOG'); break
            try:
                if now - self.feedback_at > .024: raise ValueError('joint feedback stale')
                tf = self.tf.lookup_transform('base_link', 'tcp', rclpy.time.Time())
                tf_age=(self.get_clock().now().nanoseconds-(tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))/1e9
                if not 0<=tf_age<=.024:raise ValueError('TCP transform stale')
                p, q = tf.transform.translation, tf.transform.rotation
                target = self.clutch.consume(json.loads(raw), source, time.monotonic(), [p.x,p.y,p.z],
                    Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix())
                if target is not None:
                    msg = PoseStamped(); msg.header.frame_id = 'base_link'
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target.position.tolist()
                    q = Rotation.from_matrix(target.rotation).as_quat()
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q.tolist()
                    self.publisher.publish(msg)
            except (ValueError, KeyError, TransformException, json.JSONDecodeError) as exc:
                self.clutch.invalidate(str(exc))
        self.state.publish(String(data=json.dumps(dict(held=self.clutch.held, reason=self.clutch.reason,
            age_s=now-self.clutch.received, sequence=self.clutch.sequence, software_stop=self.clutch.stopped))))


def main():
    rclpy.init(); node = QuestBridge()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.socket.close(); node.destroy_node(); rclpy.try_shutdown()
