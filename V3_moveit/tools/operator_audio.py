"""Subscriber-only visual prompts and telemetry for the desktop and headset."""
import argparse,json,time,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from operator_target import TargetHistory
from operator_guidance import live_phase
import math
ROOT=Path.cwd()
from operator_settings import atomic,RUNTIME
parser=argparse.ArgumentParser();parser.add_argument("--continuous",action="store_true");parser.add_argument("--mode",choices=["fake","hardware"],default="fake");parser.add_argument('--session-started',type=float);args=parser.parse_args()

class Cues(Node):
    def __init__(self):
        super().__init__('v3_operator_audio_cues',start_parameter_services=False,enable_rosout=False)
        self.phase=0;self.held=False;self.due=None;self.state={};self.input={};self.last_print=0.;self.ready=False;self.input_at=0.;self.state_at=0.;self.joints=[];self.joints_at=0.;self.position=[];self.velocity=[];self.sample_due=0.;self.settings_at=0.;self.rate=20
        self.followed=False;self.instruction=''
        self.create_subscription(String,"/v3/state",self.state_update,10)
        self.create_subscription(JointState,"/joint_states",self.joint_update,10)
        self.create_subscription(String,'/v3/input_state',self.inputs,10)
        self.targets=TargetHistory(time.monotonic());self.target_due=0.
        self.create_subscription(PoseStamped,'/v3/hand_target',self.target_update,1)
        self.create_timer(.008,self.tick)
        if not args.continuous:self.prompt('握住右侧 Grip，轻移手柄')
    def joint_update(self,msg):
        positions=dict(zip(msg.name,msg.position))
        values=[positions.get('joint_'+str(i)) for i in range(1,7)]
        if all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
            self.joints=[round(math.degrees(v),1) for v in values];self.joints_at=time.monotonic();self.position=values;velocities=dict(zip(msg.name,msg.velocity));self.velocity=[velocities.get("joint_"+str(i)) for i in range(1,7)]
            if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in self.velocity):self.velocity=[]
    def state_update(self,msg):
        self.state=json.loads(msg.data);self.state_at=time.monotonic()
    def target_update(self,msg):
        p=msg.pose.position;q=msg.pose.orientation
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        self.targets.pose([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],msg.header.frame_id,stamp,age,time.monotonic())
    def prompt(self,text):
        self.instruction=text
        print('OP '+json.dumps(dict(message=text,instruction=text),ensure_ascii=False),flush=True)
    def inputs(self,msg):
        d=json.loads(msg.data);self.input=d;now=time.monotonic();self.input_at=now;held=d.get('held') is True
        self.targets.inputs(d,now)
        if args.continuous:
            self.held=held;return
        if self.phase in (0,3) and held and not self.held:
            self.phase+=1;self.due=now+.5
        if self.phase in (1,2) and not held and d.get('reason')=='RELEASED':
            self.phase=6;self.due=now+3.;self.prompt('已松开，等待第二次检查')
        if self.phase in (4,5) and d.get('software_stop') is True:
            self.phase=7;self.due=None;self.prompt('已收到停止指令，松开 Grip')
        self.held=held
    def tick(self):
        now=time.monotonic()
        if now>=self.target_due:
            self.target_due=now+1/30
            atomic(ROOT/'.cache/operator/target_state.json',self.targets.snapshot(now))
        if now-self.settings_at>.5:
            self.settings_at=now
            try:self.rate=max(1,min(60,int(json.loads(RUNTIME.read_text()).get('scope_hz',20))))
            except (OSError,ValueError,TypeError):self.rate=20
        if now>=self.sample_due:
            self.sample_due=max(now-.05,self.sample_due)+1/max(20,self.rate)
            phase=live_phase(self.state,self.input,now,self.state_at,self.input_at,self.joints_at,self.followed)
            if phase=='waiting_feedback' and not self.ready:phase='starting'
            if phase=='following':self.followed=True
            if not self.ready and phase in ('ready','following'):
                self.ready=True
                if args.continuous:print('OP '+json.dumps(dict(message='遥操作已就绪。',ready=True),ensure_ascii=False),flush=True)
            atomic(ROOT/'.cache/operator/telemetry.json',dict(received_monotonic=self.joints_at,mode=args.mode,position=self.position,velocity=self.velocity,
                   input_received_monotonic=self.input_at,input_reason=self.input.get("reason"),input_age_s=self.input.get("age_s",999),
                   guide_phase=phase,guide_updated=now,session_started=args.session_started,check_instruction=self.instruction))
        if now-self.last_print>=1:
            self.last_print=now
            print("OP "+json.dumps(dict(message="运行状态："+(str(self.state.get("mode","正在启动")) if now-self.state_at<.5 else "等待新状态"),input_reason=self.input.get("reason","等待输入") if now-self.input_at<.5 else "等待新输入状态",joint_degrees=self.joints if now-self.joints_at<.5 else [],joint_feedback_fresh=now-self.joints_at<.5,control_mode=self.state.get("mode") if now-self.state_at<.5 else None),ensure_ascii=False),flush=True)
        if args.continuous:return
        if self.due is None or time.monotonic()<self.due:return
        self.due=None
        if self.phase==1:
            self.phase=2;self.prompt('现在松开 Grip')
        elif self.phase==6:
            self.phase=3;self.prompt('再次握住 Grip，轻移手柄')
        elif self.phase==4:
            self.phase=5;self.prompt('按下右摇杆，检查停止')
rclpy.init();node=Cues();end=float("inf") if args.continuous else time.monotonic()+65
owner=os.getppid()
try:
    while rclpy.ok() and time.monotonic()<end and os.getppid()==owner:rclpy.spin_once(node,timeout_sec=.05)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
