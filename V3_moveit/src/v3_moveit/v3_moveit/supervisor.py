"""协调 Servo 和 OMPL；只有本节点能把轨迹交给 arm_controller。

正常时连续跟随。接近障碍且手柄目标稳定后，先停稳，再异步规划绕行。
目标或场景变化使旧规划结果失效，并取消正在执行的轨迹。规划过程不会占用 JAKA 的发送线程。
"""
import copy
import json
import hashlib
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import ServoStatus, PlanningScene, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import ServoCommandType, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from rosidl_runtime_py.convert import message_to_ordereddict
from .coordinator import Epoch


def pose_vector(msg):
    p=msg.pose.position; q=msg.pose.orientation
    return np.array([p.x,p.y,p.z]), Rotation.from_quat([q.x,q.y,q.z,q.w])


def static_message_data(message):
    """Preserve collision data and frame IDs, ignoring only transport stamps."""
    def clean(value):
        if isinstance(value,dict):
            return {key:clean(item) for key,item in value.items() if key!='stamp'}
        if isinstance(value,(list,tuple)):return [clean(item) for item in value]
        return value
    return clean(message_to_ordereddict(message))


class SceneEnvironment:
    """Canonical non-motion content of the monitored PlanningScene.

    Diff/full rules follow MoveIt 2.12.4 planning_scene.cpp. In that version
    padding, scale and fixed transforms update keys even in a full message;
    world and ACM are replaced. RobotState.is_diff governs attached bodies.
    This is an invalidation fingerprint, not a second collision scene or TF engine.
    """
    def __init__(self):
        self.parts=dict(world={},attached={},acm=dict(entries={},defaults={}),
                        padding={},scale={},transforms={},octomap=None,model='')
        self.signature=self.fingerprint(self.parts)

    @staticmethod
    def fingerprint(parts):
        data=json.dumps(parts,sort_keys=True,separators=(',',':'),allow_nan=False)
        return hashlib.sha256(data.encode()).hexdigest()

    @staticmethod
    def collision_update(objects,obj):
        if obj.operation==obj.REMOVE:
            if obj.id:objects.pop(obj.id,None)
            else:objects.clear()  # MoveIt REMOVE with an empty ID removes all world objects.
            return
        if not obj.id:raise ValueError('collision object has no ID')
        incoming=static_message_data(obj);incoming.pop('operation')
        old=objects.get(obj.id)
        if obj.operation==obj.ADD or (obj.operation==obj.APPEND and old is None):
            objects[obj.id]=incoming
        elif obj.operation==obj.MOVE:
            if old is None:raise ValueError('MOVE references an unknown collision object')
            old['header']=incoming['header'];old['pose']=incoming['pose']
            for field in ('primitive_poses','mesh_poses','plane_poses'):
                if incoming[field]:old[field]=incoming[field]
        elif obj.operation==obj.APPEND:
            for field in ('primitives','primitive_poses','meshes','mesh_poses','planes','plane_poses'):
                old[field].extend(incoming[field])
            # Keep subframes keyed by name so repeated ordering does not matter.
            subframes=dict(zip(old['subframe_names'],old['subframe_poses']))
            subframes.update(zip(incoming['subframe_names'],incoming['subframe_poses']))
            old['subframe_names']=sorted(subframes)
            old['subframe_poses']=[subframes[name] for name in old['subframe_names']]
        else:raise ValueError('unknown collision object operation')

    @staticmethod
    def acm_data(acm):
        names=list(acm.entry_names)
        if (len(set(names))!=len(names) or len(acm.entry_values)!=len(names) or
                any(len(row.enabled)!=len(names) for row in acm.entry_values) or
                len(acm.default_entry_names)!=len(acm.default_entry_values)):
            raise ValueError('malformed allowed collision matrix')
        return dict(entries={name:{other:bool(acm.entry_values[i].enabled[j])
                                  for j,other in enumerate(names)} for i,name in enumerate(names)},
                    defaults=dict(zip(acm.default_entry_names,acm.default_entry_values)))

    def update(self,msg):
        full=not msg.is_diff;state=msg.robot_state;acm=msg.allowed_collision_matrix
        state_present=full or bool(state.joint_state.name or state.multi_dof_joint_state.joint_names or state.attached_collision_objects)
        clear_attached=state_present and not state.is_diff
        octomap=msg.world.octomap
        if not (full or msg.world.collision_objects or octomap.octomap.id or octomap.octomap.data or
                state.attached_collision_objects or (clear_attached and self.parts['attached']) or
                acm.entry_names or acm.default_entry_names or msg.link_padding or msg.link_scale or
                msg.fixed_frame_transforms or msg.robot_model_name):
            return False  # Joint feedback/timestamps alone do not hash the static scene.
        parts=copy.deepcopy(self.parts)
        if full:parts['world']={}
        if clear_attached:parts['attached']={}
        for attached in state.attached_collision_objects:
            obj=attached.object
            if obj.operation==obj.REMOVE:
                ids=[name for name,value in parts['attached'].items()
                     if (not obj.id or name==obj.id) and (not attached.link_name or value['link_name']==attached.link_name)]
                for name in ids:parts['attached'].pop(name)
                continue
            data=static_message_data(attached)
            data['touch_links']=sorted(set(data['touch_links']))
            old=parts['attached'].get(obj.id)
            objects={obj.id:copy.deepcopy(old['object'])} if old else {}
            self.collision_update(objects,obj)
            data['object']=objects[obj.id]
            parts['attached'][obj.id]=data
            parts['world'].pop(obj.id,None)  # Attaching transfers a world object's ownership.
        for obj in msg.world.collision_objects:self.collision_update(parts['world'],obj)
        if full or acm.entry_names or acm.default_entry_names:
            parts['acm']=self.acm_data(acm)
        for item in msg.link_padding:
            if item.padding==0.:parts['padding'].pop(item.link_name,None)
            else:parts['padding'][item.link_name]=item.padding
        for item in msg.link_scale:
            if item.scale==1.:parts['scale'].pop(item.link_name,None)
            else:parts['scale'][item.link_name]=item.scale
        for transform in msg.fixed_frame_transforms:
            # MoveIt stores transforms by source frame, with child=the model frame.
            parts['transforms'][transform.header.frame_id]=static_message_data(transform)
        if full or octomap.octomap.id or octomap.octomap.data:
            # A nonempty ID with empty data explicitly clears an octomap diff.
            parts['octomap']=static_message_data(octomap) if octomap.octomap.data else None
        if msg.robot_model_name:parts['model']=msg.robot_model_name
        signature=self.fingerprint(parts)
        changed=signature!=self.signature
        self.parts=parts;self.signature=signature
        return changed


class Supervisor(Node):
    def __init__(self):
        super().__init__('v3_supervisor')
        self.declare_parameter('auto_replan',True)
        self.declare_parameter('planning_time',.3)
        self.declare_parameter('mode','fake')
        self.epoch=Epoch(); self.mode='BOOT'; self.goal=None;self.reference_goal=None
        self.goal_at=-float('inf');self.stable_at=time.monotonic();self.input_at=-float('inf')
        self.joint_at=-float('inf');self.scene_at=-float('inf');self.servo_at=-float('inf');self.trajectory_at=-float('inf')
        self.held=False;self.input_seen=False;self.scene_ready=False;self.code=-1;self.q=None;self.v=None
        self.quiet_since=None;self.brake_started=0.;self.have_moved=False
        self.command_q=None;self.command_at=-float('inf');self.command_stamp_ns=None
        self.command_quiet_since=None;self.command_quiet_q=None;self.plan_start_command=None
        self.ticket=None;self.plan_future=None;self.execute_handle=None;self.cancel_pending=None
        self.execute_future=None;self.execute_result_future=None
        self.last_replan=-float('inf');self.service_future=None;self.resume_started=0.;self.scene_version=''
        self.scene_environment=SceneEnvironment()
        self.out=self.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory',1)
        self.pose_pub=self.create_publisher(PoseStamped,'/v3/pose_target',1)
        self.lease=self.create_publisher(Bool,'/v3/motion_lease',1)
        self.state=self.create_publisher(String,'/v3/state',1)
        self.create_subscription(PoseStamped,'/v3/hand_target',self.hand,1)
        self.create_subscription(String,'/v3/input_state',self.input,1)
        self.create_subscription(String,'/v3/controller_fault',self.controller_fault,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState,'/joint_states',self.joints,1)
        self.create_subscription(JointTrajectoryControllerState,'/arm_controller/controller_state',self.command_state,1)
        self.create_subscription(ServoStatus,'/v3/servo_status',self.status,1)
        self.create_subscription(JointTrajectory,'/v3/servo_trajectory',self.trajectory,1)
        self.create_subscription(String,'/v3/scene_ready',self.scene,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PlanningScene,'/monitored_planning_scene',self.scene_update,10)
        self.switch=self.create_client(ServoCommandType,'/servo_node/switch_command_type')
        self.pause=self.create_client(SetBool,'/servo_node/pause_servo')
        self.plan=self.create_client(GetMotionPlan,'/plan_kinematic_path')
        self.execute=ActionClient(self,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
        self.create_timer(.008,self.tick)

    def hand(self,msg):
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if msg.header.frame_id!='base_link' or not 0<=age<=.08:return
        now=time.monotonic(); p,r=pose_vector(msg)
        if self.reference_goal is None:self.reference_goal=copy.deepcopy(msg);self.stable_at=now
        else:
            rp,rr=pose_vector(self.reference_goal)
            if np.linalg.norm(p-rp)>.002 or (r*rr.inv()).magnitude()>.01745:
                self.reference_goal=copy.deepcopy(msg);self.stable_at=now;self.epoch.invalidate()
                if self.mode in ('PLANNING','EXECUTING','PAUSING'):self.cancel('hand target changed')
        self.goal=msg;self.goal_at=now
        if self.mode=='FOLLOW' and self.held:self.pose_pub.publish(msg)

    def input(self,msg):
        try:d=json.loads(msg.data)
        except ValueError:return
        now=time.monotonic();previous=self.held
        self.held=d.get('held') is True and d.get('age_s',999)<=.08 and not d.get('software_stop',False)
        self.input_at=now
        self.input_seen |= d.get('age_s',999)<=.08
        if previous and not self.held:
            self.epoch.invalidate();self.cancel('grip released or input invalid')

    def joints(self,msg):
        try:
            indices=[msg.name.index(f'joint_{i}') for i in range(1,7)]
            q=np.array([msg.position[i] for i in indices]);v=np.array([msg.velocity[i] for i in indices])
            age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
            if not np.isfinite(q).all() or not np.isfinite(v).all() or not 0<=age<=.024:return
            self.q=q;self.v=v;self.joint_at=time.monotonic()
            if np.max(np.abs(v))<.002:
                if self.quiet_since is None:self.quiet_since=self.joint_at
            else:self.quiet_since=None
        except (ValueError,IndexError):return

    def command_state(self,msg):
        """JTC output is the sent target, not physical feedback.

        Position-only output has no v/a. A stationary reference must therefore
        coincide with that output before its v/a can establish a planning start.
        Header time is the controller's target clock; receipt time remains host
        monotonic time and is independently bounded.
        """
        now=time.monotonic()
        try:
            names=[f'joint_{i}' for i in range(1,7)]
            if len(msg.joint_names)!=6 or set(msg.joint_names)!=set(names):raise ValueError('command joints')
            indices=[msg.joint_names.index(name) for name in names]
            fields=(msg.output.positions,msg.reference.positions,msg.reference.velocities,msg.reference.accelerations)
            if any(len(values)!=6 or not np.isfinite(values).all() for values in fields):raise ValueError('command fields')
            q,reference,v,a=(np.array([values[i] for i in indices]) for values in fields)
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            age=(self.get_clock().now().nanoseconds-stamp)/1e9
            if not -.024<=age<=.024 or (self.command_stamp_ns is not None and stamp<=self.command_stamp_ns):
                raise ValueError('command target timestamp')
            uninterrupted=0<=now-self.command_at<.024
            stationary=(np.max(np.abs(reference-q))<=1e-10 and
                        np.max(np.abs(v))<=1e-8 and np.max(np.abs(a))<=1e-8)
            if stationary:
                if (not uninterrupted or self.command_quiet_since is None or self.command_quiet_q is None or
                        np.max(np.abs(q-self.command_quiet_q))>1e-10):
                    self.command_quiet_since=now;self.command_quiet_q=q.copy()
            else:self.command_quiet_since=None;self.command_quiet_q=None
            self.command_q=q;self.command_at=now;self.command_stamp_ns=stamp
        except (ValueError,IndexError,TypeError):
            self.command_q=None;self.command_at=-float('inf')
            self.command_quiet_since=None;self.command_quiet_q=None

    def command_ready(self,expected=None):
        now=time.monotonic()
        if (self.command_q is None or self.command_stamp_ns is None or self.command_quiet_since is None or
                not 0<=now-self.command_at<.024 or now-self.command_quiet_since<.08):return False
        # A target clock may lead or lag ROS time slightly; it is never treated
        # as the robot feedback's acquisition timestamp.
        age=(self.get_clock().now().nanoseconds-self.command_stamp_ns)/1e9
        if not -.024<=age<=.024:return False
        return expected is None or np.max(np.abs(self.command_q-expected))<=1e-10

    def physical_stopped(self):
        now=time.monotonic()
        return (self.q is not None and self.v is not None and self.quiet_since is not None and
                0<=now-self.joint_at<.024 and now-self.quiet_since>.08 and np.max(np.abs(self.v))<.002)

    def status(self,msg):self.code=msg.code;self.servo_at=time.monotonic()
    def scene(self,msg):
        try:d=json.loads(msg.data)
        except ValueError:return
        self.scene_ready=d.get('ready') is True;self.scene_at=time.monotonic()
        if not self.scene_ready and self.mode!='BOOT':self.fault('scene unavailable')
    def scene_update(self,msg):
        try:changed=self.scene_environment.update(msg)
        except (ValueError,TypeError,KeyError) as exc:
            self.fault(f'cannot verify planning scene environment: {exc}');return
        if not changed:return
        self.scene_version=self.scene_environment.signature
        self.epoch.scene_changed()
        if self.mode in ('FOLLOW','BRAKING','EXECUTING','CANCELING'):
            # Both queued Servo knots and a prechecked action stop become invalid
            # on environment changes, even before physical feedback starts moving.
            self.fault('planning scene changed while a motion source owns the target queue')
        elif self.mode in ('PLANNING','PAUSING'):
            self.cancel('planning scene changed')

    def trajectory(self,msg):
        self.trajectory_at=time.monotonic()
        if self.mode not in ('FOLLOW','BRAKING','IDLE'):return
        if len(msg.joint_names)!=6 or set(msg.joint_names)!={f'joint_{i}' for i in range(1,7)}:return
        if not msg.points or any(len(p.positions)!=6 or not np.isfinite(p.positions).all() for p in msg.points):return
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if not -.04<=age<=.08:return
        self.out.publish(msg)

    def cancel(self,reason):
        if self.mode in ('BOOT','FAULT','IDLE'):return
        self.get_logger().info(reason)
        self.epoch.invalidate()
        previous=self.mode
        if self.execute_handle is not None:
            try:self.cancel_pending=self.execute_handle.cancel_goal_async()
            except Exception as exc:self.fault(f'action cancellation failed: {exc}');return
            self.execute_handle=None
        if previous not in ('CANCELING','BRAKING'):self.brake_started=time.monotonic()
        self.mode='CANCELING' if previous in ('PLANNING','EXECUTING','PAUSING','CANCELING') else 'BRAKING'

    def resume(self):
        if self.mode=='FAULT':return
        if self.execute_future is not None or self.execute_result_future is not None:return
        if self.mode!='RESUMING':
            self.mode='RESUMING';self.resume_started=time.monotonic();self.service_future=None
        self.start_resume_if_ready()

    def start_resume_if_ready(self):
        # The action result confirms the terminal command was emitted, not that
        # command and physical feedback have both remained stationary for 80 ms.
        if self.mode!='RESUMING' or self.service_future is not None:return
        if not self.command_ready() or not self.physical_stopped() or not self.pause.service_is_ready():return
        try:self.service_future=self.pause.call_async(SetBool.Request(data=False))
        except Exception as exc:self.fault(f'Servo resume request failed: {exc}')

    def begin_plan(self):
        if self.mode=='FAULT' or self.plan_future is not None:return
        if not self.command_ready() or not self.physical_stopped():return
        if self.execute_future is not None or self.execute_result_future is not None:return
        if not self.plan.service_is_ready() or not self.execute.server_is_ready() or not self.pause.service_is_ready():return
        self.ticket=self.epoch.ticket(self.q);self.plan_goal=copy.deepcopy(self.goal)
        self.plan_start_command=self.command_q.copy()
        self.mode='PAUSING';self.service_future=self.pause.call_async(SetBool.Request(data=True))
        self.last_replan=time.monotonic()

    def send_plan(self):
        if self.mode=='FAULT':return
        if (self.plan_start_command is None or not self.command_ready(self.plan_start_command) or
                not self.physical_stopped() or not self.epoch.accepts(self.ticket,self.q,self.held)):
            self.get_logger().info('discarded planning request: commanded start is no longer stationary and fresh')
            self.resume();return
        request=GetMotionPlan.Request();req=request.motion_plan_request
        req.group_name='arm';req.pipeline_id='ompl';req.planner_id='RRTConnectkConfigDefault'
        req.allowed_planning_time=float(self.get_parameter('planning_time').value)
        req.num_planning_attempts=1;req.max_velocity_scaling_factor=1.;req.max_acceleration_scaling_factor=1.
        req.start_state.is_diff=True;req.start_state.joint_state.name=[f'joint_{i}' for i in range(1,7)]
        req.start_state.joint_state.position=self.plan_start_command.tolist()
        req.start_state.joint_state.velocity=[0.]*6
        c=Constraints();pc=PositionConstraint();pc.header=self.plan_goal.header;pc.link_name='tcp';pc.weight=1.
        region=SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.001])
        pc.constraint_region.primitives=[region];pc.constraint_region.primitive_poses=[copy.deepcopy(self.plan_goal.pose)]
        oc=OrientationConstraint();oc.header=self.plan_goal.header;oc.link_name='tcp';oc.orientation=self.plan_goal.pose.orientation
        oc.absolute_x_axis_tolerance=.01;oc.absolute_y_axis_tolerance=.01;oc.absolute_z_axis_tolerance=.01;oc.weight=1.
        c.position_constraints=[pc];c.orientation_constraints=[oc];req.goal_constraints=[c]
        self.mode='PLANNING';ticket=self.ticket
        # A pending service must finish before another request is admitted. The planner's
        # allowed_planning_time is a planning budget, not an end-to-end response deadline.
        self.plan_future=self.plan.call_async(request)
        self.plan_future.add_done_callback(lambda result:self.planned(result,ticket))

    def planned(self,future,ticket):
        if future is not self.plan_future:return
        self.plan_future=None
        if (self.mode!='PLANNING' or not self.epoch.accepts(ticket,self.q,self.held) or
                self.plan_start_command is None or not self.command_ready(self.plan_start_command) or
                not self.physical_stopped()):
            self.get_logger().info('discarded stale planning result')
            if self.mode=='PLANNING':self.resume()
            return
        try:result=future.result().motion_plan_response
        except Exception as exc:
            self.get_logger().error(f'planning service failed: {exc}');self.resume();return
        traj=result.trajectory.joint_trajectory
        if result.error_code.val!=1 or not traj.points:self.resume();return
        if len(traj.joint_names)!=6 or set(traj.joint_names)!={f'joint_{i}' for i in range(1,7)}:self.fault('planner joint names');return
        if any(len(values)!=6 or not np.isfinite(values).all()
               for p in traj.points for values in (p.positions,p.velocities,p.accelerations)):
            self.fault('invalid planned trajectory');return
        if any(max(abs(x) for x in values)>1e-8 for p in (traj.points[0],traj.points[-1])
               for values in (p.velocities,p.accelerations)):
            self.fault('planned trajectory must start and finish at rest');return
        times=[p.time_from_start.sec*10**9+p.time_from_start.nanosec for p in traj.points]
        if times[0]!=0 or any(b<=a for a,b in zip(times,times[1:])):
            self.fault('invalid planned trajectory times');return
        if np.max(np.abs(np.array(traj.points[0].positions)-np.array([self.plan_start_command[int(n.split('_')[-1])-1] for n in traj.joint_names])))>1e-10:
            self.resume();return
        # JTC starts at its first update; transport latency must not skip the ramp.
        traj.header.stamp.sec=0;traj.header.stamp.nanosec=0
        self.mode='EXECUTING'
        goal=FollowJointTrajectory.Goal();goal.trajectory=traj
        def accepted(f):
            if f is self.execute_future:self.execute_future=None
            try:handle=f.result()
            except Exception as exc:self.fault(f'action acceptance failed: {exc}');return
            if not handle.accepted:
                if self.mode=='EXECUTING' and ticket.generation==self.epoch.generation:self.resume()
                return
            self.execute_handle=handle
            try:
                self.execute_result_future=handle.get_result_async()
                self.execute_result_future.add_done_callback(self.executed)
            except Exception as exc:
                self.fault(f'action result registration failed: {exc}');return
            if self.execute_result_future is None:return
            if self.mode!='EXECUTING' or ticket.generation!=self.epoch.generation:
                self.execute_handle=None
                try:self.cancel_pending=handle.cancel_goal_async()
                except Exception as exc:self.fault(f'late action cancellation failed: {exc}')
                return
        try:
            self.execute_future=self.execute.send_goal_async(goal)
            self.execute_future.add_done_callback(accepted)
        except Exception as exc:self.execute_future=None;self.fault(f'action submission failed: {exc}')

    def executed(self,result):
        if result is not self.execute_result_future:return
        self.execute_result_future=None;self.execute_handle=None
        try:
            outcome=result.result()
            if outcome.status not in (4,5) or outcome.result.error_code!=0:
                self.fault('trajectory controller did not complete successfully');return
        except Exception as exc:self.fault(f'action result failed: {exc}');return
        if self.mode=='EXECUTING':self.resume()

    def controller_fault(self,msg):
        self.fault(f'controller fault: {msg.data}')

    def fault(self,reason):
        # Revoke the hardware lease before any fallible action/service operation.
        self.mode='FAULT';self.epoch.invalidate();self.lease.publish(Bool(data=False))
        self.get_logger().error(reason)
        if self.execute_handle is not None:
            handle=self.execute_handle;self.execute_handle=None
            try:self.cancel_pending=handle.cancel_goal_async()
            except Exception as exc:self.get_logger().error(f'action cancellation after fault failed: {exc}')

    def tick(self):
        now=time.monotonic();quiet=self.quiet_since is not None and now-self.quiet_since>.08
        healthy=self.scene_ready and now-self.scene_at<.8 and now-self.joint_at<.024
        if self.mode=='BOOT':
            if self.service_future is None and self.switch.service_is_ready():
                self.service_future=self.switch.call_async(ServoCommandType.Request(command_type=2))
            if self.service_future is not None and self.service_future.done():
                if not self.service_future.result().success:self.fault('Servo POSE mode rejected');return
                if healthy:self.mode='IDLE';self.service_future=None
        elif self.mode!='FAULT' and not healthy:self.fault('scene or joint feedback expired')
        if self.held and (now-self.input_at>.08 or now-self.goal_at>.08):
            self.held=False;self.epoch.invalidate();self.cancel('input expired')
        if self.mode=='IDLE' and self.held and self.goal is not None:
            self.mode='FOLLOW';self.pose_pub.publish(self.goal)
        if self.mode=='FOLLOW':
            if now-self.servo_at>.08:self.fault('Servo heartbeat expired')
            elif self.code==-1:self.fault('Servo rejected the current command')
            elif (self.code in (4,5) and self.get_parameter('auto_replan').value
                  and now-self.stable_at>.25 and now-self.last_replan>1.):
                self.mode='BRAKING';self.brake_started=now
        if self.mode=='BRAKING' and quiet and now-self.brake_started>.12:
            if self.held and now-self.stable_at>.25:self.begin_plan()
            elif not self.held:self.mode='IDLE'
            else:self.mode='FOLLOW'
        if self.mode=='RESUMING':
            if now-self.resume_started>1.5:self.fault('stationary Servo resume not confirmed')
            else:self.start_resume_if_ready()
        if self.mode in ('PAUSING','RESUMING') and self.service_future is not None and self.service_future.done():
            if not self.service_future.result().success:self.fault('Servo pause/resume failed');return
            self.service_future=None
            if self.mode=='PAUSING':
                if self.epoch.accepts(self.ticket,self.q,self.held):self.send_plan()
                else:self.resume()
            else:self.mode='IDLE'
        if (self.mode=='CANCELING' and self.execute_future is None and self.execute_result_future is None
                and (self.cancel_pending is None or self.cancel_pending.done())
                and (self.service_future is None or self.service_future.done()) and quiet):
            self.cancel_pending=None;self.resume()
        if self.mode in ('BRAKING','CANCELING') and now-self.brake_started>1.5:self.fault('controlled stop not confirmed')
        # 空闲时保持当前静止关节；输入中断后先完成有界制动，不能无限执行旧计划。
        self.lease.publish(Bool(data=healthy and self.input_seen and self.mode not in ('BOOT','FAULT')))
        self.state.publish(String(data=json.dumps(dict(mode=self.mode,servo_code=self.code,held=self.held,
            feedback_age_ms=(now-self.joint_at)*1000,scene_ready=self.scene_ready,auto_replan=self.get_parameter('auto_replan').value))))


def main():
    rclpy.init();node=Supervisor()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    except RuntimeError as exc:
        # Jazzy may invalidate a subscription handle while its default SIGINT
        # handler shuts down the context. Only this exact conversion race after
        # context shutdown is a cleanup condition; live/runtime errors propagate.
        if node.context.ok() or str(exc) != "Unable to convert call argument '0' to Python object (#define PYBIND11_DETAILED_ERROR_MESSAGES or compile in debug mode for details)":
            raise
    finally:
        if node.context.ok():node.lease.publish(Bool(data=False))
        node.destroy_node();rclpy.try_shutdown()
