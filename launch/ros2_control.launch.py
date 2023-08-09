from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,Command,FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    iiwa_controllers = PathJoinSubstitution([FindPackageShare('wbc_ros'), 'config', 'iiwa_controllers.yaml'])
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),' ',
        PathJoinSubstitution([FindPackageShare('wbc_ros'), 'models', 'urdf', 'iiwa.config.xacro']),' ',
        'prefix:=',                          '/',                      ' ',
        'use_sim:=',                         'false',                  ' ',
        'use_fake_hardware:=',               'true',                   ' ',
        'initial_positions:=',               'initial_positions.yaml', ' ',
        'command_interface:=',               'position',               ' ',
        'namespace:=',                       '/'])
    robot_description = {'robot_description': robot_description}

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, iiwa_controllers],
            output='both',
            namespace='/'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['whole_body_controller', '--controller-manager', ['/', 'controller_manager']],
            parameters=[{'robot_model.file': robot_description}],
            namespace='/'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', ['/', 'controller_manager']],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='/',
            output='both',
            parameters=[robot_description],
        )
    ])
