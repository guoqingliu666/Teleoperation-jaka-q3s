"""Ubuntu 24.04 / Jazzy。默认 fake；hardware 必须显式选取，不修改柜端工具或负载。"""
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from scipy.spatial.transform import Rotation
from v3_moveit.launch_policy import servo_config, tracking_config
from v3_moveit.tool_config import apply_tool, tool_values
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    root=Path(get_package_share_directory('v3_moveit'))
    mode=LaunchConfiguration('mode').perform(context)
    if mode not in ('fake','hardware'):raise ValueError('mode must be fake or hardware')
    fake_tracking=LaunchConfiguration('fake_velocity_tracking',default='false').perform(context)=='true'
    servo_parameters=servo_config(yaml.safe_load((root/'config/servo.yaml').read_text()), mode,
        LaunchConfiguration('fake_self_collision_proximity_threshold',default='').perform(context))
    guarded=LaunchConfiguration('guarded_fake').perform(context)=='true'
    if guarded and mode!='fake':raise ValueError('guarded_fake is a test backend for mode=fake only')
    audit=LaunchConfiguration('guard_audit_path').perform(context)
    if guarded and not Path(audit).is_absolute():raise ValueError('guard_audit_path must be absolute')
    cfg=yaml.safe_load((root/'config/runtime.yaml').read_text())
    local=LaunchConfiguration('runtime_config').perform(context)
    if local:cfg.update(yaml.safe_load(Path(local).read_text()))
    velocity_tracking=tracking_config(cfg,mode,fake_tracking)
    meta=json.loads((root/'config/model_metadata.json').read_text())
    # Some diagnostics call setup() directly, before DeclareLaunchArgument runs.
    payload_path=LaunchConfiguration('payload_config',default=os.environ.get('V3_PAYLOAD_CONFIG','')).perform(context)
    payload=json.loads((Path(payload_path) if payload_path else root/'config/payload.json').read_text())
    urdf=ET.parse(root/'description/s5.urdf').getroot()
    urdf,meta=apply_tool(urdf,meta,payload)
    tool_id,tcp=tool_values(payload)
    control=ET.SubElement(urdf,'ros2_control',name='JakaS5',type='system')
    hw=ET.SubElement(control,'hardware')
    ET.SubElement(hw,'plugin').text=('v3_moveit/GuardedGenericSystem' if guarded else 'mock_components/GenericSystem') if mode=='fake' else 'v3_moveit/JakaSystem'
    params={'calculate_dynamics':'true','allow_hardware':str(mode=='hardware').lower(),
        'robot_ip':cfg['robot_ip'],'edg_client_ip':cfg['edg_client_ip'],'edg_port':str(cfg['edg_port']),
        'payload_kg':str(payload['payload_kg']),'tool_id':str(tool_id)}
    for i in range(6):params[f'tool_tcp_{i}']=str(tcp[i])
    if guarded:params.update(audit_path=audit,strict=LaunchConfiguration('guard_strict').perform(context))
    for i in range(3):params[f'cog_{i}']=str(payload['cog_mm'][i])
    for i in range(6):
        params[f'lower_{i}']=str(meta['joint_lower'][i]);params[f'upper_{i}']=str(meta['joint_upper'][i])
        params[f'fk_q_{i}']=str(meta['q_reference_rad'][i])
    tcp_home=meta['tcp_home']
    expected=[tcp_home[i][3]*1000 for i in range(3)]+Rotation.from_matrix([r[:3] for r in tcp_home[:3]]).as_euler('xyz').tolist()
    for i in range(6):params[f'fk_tcp_{i}']=str(expected[i])
    for key,value in params.items():ET.SubElement(hw,'param',name=key).text=value
    for i in range(6):
        joint=ET.SubElement(control,'joint',name=f'joint_{i+1}')
        ET.SubElement(joint,'command_interface',name='position')
        pos=ET.SubElement(joint,'state_interface',name='position')
        ET.SubElement(pos,'param',name='initial_value').text=str(meta['q_reference_rad'][i])
        vel=ET.SubElement(joint,'state_interface',name='velocity')
        ET.SubElement(vel,'param',name='initial_value').text='0.0'
    description={'robot_description':ET.tostring(urdf,encoding='unicode')}
    semantic={'robot_description_semantic':(root/'description/s5.srdf').read_text()}
    kinematics={'robot_description_kinematics':{'arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_timeout':.005,'kinematics_solver_search_resolution':.005}}}
    limits={'robot_description_planning':yaml.safe_load((root/'config/joint_limits.yaml').read_text())}
    common=[description,semantic,kinematics,limits]
    pipeline={'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
        'ompl':{'planning_plugins':['ompl_interface/OMPLPlanner'],
            'request_adapters':['default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters':['v3_moveit/PathRuckigRetiming',
                'default_planning_response_adapters/ValidateSolution'],
            'planner_configs':{'RRTConnectkConfigDefault':{'type':'geometric::RRTConnect','range':0.0}},
            'arm':{'planner_configs':['RRTConnectkConfigDefault'],'longest_valid_segment_fraction':.005}}}
    nodes=[Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[description,{'publish_frequency':125.0}],output='screen')]
    manager=Node(package='controller_manager',executable='ros2_control_node',
        parameters=[str(root/'config/controllers.yaml')],remappings=[('robot_description','/robot_description')],output='screen')
    nodes.append(manager)
    nodes.extend([Node(package='controller_manager',executable='spawner',arguments=[name,'--controller-manager-timeout','60'],output='screen')
        for name in ('joint_state_broadcaster','arm_controller')])
    move_group=Node(package='moveit_ros_move_group',executable='move_group',additional_env={'DEBUGINFOD_URLS':''},parameters=common+[pipeline,
        {'allow_trajectory_execution':False,'publish_robot_description_semantic':True,
         'disable_capabilities':'move_group/MoveGroupMoveAction move_group/MoveGroupExecuteTrajectoryAction',
         'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,
         'publish_transforms_updates':True}],output='screen')
    # Upstream emits singularity warnings every Servo tick. Synchronous log sinks
    # can block the command loop; status codes/messages still publish at 125 Hz.
    # Keep errors from that logger and all adapter/controller fault reporting.
    servo=Node(package='v3_moveit',executable='v3_fixed_servo_node',name='servo_node',
        arguments=['--ros-args','--log-level','servo_node.moveit.ros.servo:=error',
                    '--log-level','moveit.ros.servo:=error'],
        parameters=common+[{'moveit_servo':servo_parameters,
            'planning_group_name':'arm','update_period':.008,'runtime_mode':mode,
            'velocity_tracking':velocity_tracking}],output='screen')
    supervisor=Node(package='v3_moveit',executable='supervisor',parameters=[{
        'auto_replan':bool(cfg['auto_replan']),'planning_time':float(cfg['planning_time_s']),'mode':mode}],output='screen')
    nodes.extend([move_group,servo,supervisor,
        Node(package='v3_moveit',executable='scene_loader',parameters=[{'scene':cfg['scene']}],output='screen'),
        Node(package='v3_moveit',executable='quest_bridge',parameters=[{'bind':cfg['quest_bind'],
            'port':int(cfg['quest_port']),'basis':[float(x) for row in cfg['tracking_to_robot_basis'] for x in row]}],output='screen')])
    if LaunchConfiguration('rviz').perform(context)=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/view.rviz')],parameters=common,output='screen'))
    # 核心进程退出即退出整套控制；不留下一个继续执行旧计划的孤立控制器。
    for process in (manager,move_group,servo,supervisor):
        nodes.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='V3 core process exited'))])))
    nodes.append(LogInfo(msg=f"Servo collision checks ON; self proximity={servo_parameters['self_collision_proximity_threshold']} m, scene proximity={servo_parameters['scene_collision_proximity_threshold']} m, mode={mode}"))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='fake'),
        DeclareLaunchArgument('payload_config',default_value=os.environ.get('V3_PAYLOAD_CONFIG','')),
        DeclareLaunchArgument('fake_self_collision_proximity_threshold',default_value=''),
        DeclareLaunchArgument('fake_velocity_tracking',default_value='false'),
        DeclareLaunchArgument('guarded_fake',default_value='false'),
        DeclareLaunchArgument('guard_audit_path',default_value=''),DeclareLaunchArgument('guard_strict',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('runtime_config',default_value=''),OpaqueFunction(function=setup)])
