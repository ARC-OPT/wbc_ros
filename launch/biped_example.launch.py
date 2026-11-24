import sys
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import RegisterEventHandler, EmitEvent, TimerAction
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
        
    config_file_path = os.path.join('src', 'wbc_ros', 'config', 'hyper_2_example', 'hyper_2_controllers.yaml')
    with open(config_file_path, 'r') as f:
        config = yaml.safe_load(f)
    urdf_path = config['/**']['ros__parameters']['robot_urdf_path']
    
    controller = Node(
        package='wbc_ros',
        executable='biped_controller',
        parameters=[config_file_path,
                    {'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)}],
        remappings=[('/whole_body_control/robot_state',   '/mujoco/robot_state'),
                    ('/whole_body_control/solver_output', '/mujoco/joint_command')],
        output='screen'
    )
    return LaunchDescription([controller])
