from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('hand_control_service'),
        'config',
        'service.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the hand control service parameter file',
        ),
        Node(
            package='hand_control_service',         # 包名
            executable='hand_control_service',  # 可执行文件名
            name='hand_control_service', # 节点名称
            namespace='/hand_control',      # 设置命名空间
            output='screen',                 # 输出日志到终端
            emulate_tty=True,                 # 更好地显示日志            
            parameters=[LaunchConfiguration('params_file')],
        )
    ])
