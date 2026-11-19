"""仅对 fake 后端发送一段小幅模拟手柄轨迹，禁止在 hardware 模式运行。"""
import json
import socket
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient


def main():
    rclpy.init();node=Node('synthetic_quest')
    client=AsyncParameterClient(node,'/v3_supervisor')
    if not client.wait_for_services(timeout_sec=30):raise RuntimeError('supervisor unavailable')
    future=client.get_parameters(['mode']);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
    if not future.done() or future.result().values[0].string_value!='fake':raise RuntimeError('synthetic input requires fake mode')
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);start=time.monotonic();seq=0
    try:
        while time.monotonic()-start<9:
            now=time.monotonic();elapsed=now-start;held=2<elapsed<7
            distance=min(.02,max(0.,elapsed-2)*.004) if held else 0.
            packet=dict(sequence=seq,unity_time_s=now,right=dict(connected=True,tracked=True,pose_valid=True,
                position_m=[0,0,distance],rotation_xyzw=[0,0,0,1],grip=1. if held else 0.,thumbstick_click=False))
            sock.sendto(json.dumps(packet).encode(),('127.0.0.1',5006));seq+=1
            rclpy.spin_once(node,timeout_sec=.01)
    finally:sock.close();node.destroy_node();rclpy.shutdown()
