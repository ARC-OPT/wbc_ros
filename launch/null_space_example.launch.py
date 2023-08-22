from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,Command,FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
import os

def generate_launch_description():
    iiwa_controllers = PathJoinSubstitution([FindPackageShare('wbc_ros'), 'config', 'null_space_example', 'iiwa_controllers.yaml'])
    trajectory_publisher_config = os.path.join(get_package_share_directory('wbc_ros'),'config','null_space_example','trajectory_publisher.yml')
    pose_publisher_config = os.path.join(get_package_share_directory('wbc_ros'),'config','null_space_example','pose_publisher.yml')

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

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, iiwa_controllers],
        output='both',
        namespace='/'
    )
    whole_body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['whole_body_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/')

    ee_pose_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ee_pose_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/')

    elbow_pose_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['elbow_pose_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/')

    # Delay start of elbow_pose_controller_spawner after `whole_body_controller_spawner` to have the reference interfaces in WBC ready
    delayed_elbow_pose_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=whole_body_controller_spawner,
                on_exit=[elbow_pose_controller_spawner])))

    # Delay start of ee_pose_controller_spawner after `whole_body_controller_spawner` to have the reference interfaces in WBC ready
    delayed_ee_pose_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=whole_body_controller_spawner,
                on_exit=[ee_pose_controller_spawner])))

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', ['/', 'controller_manager']])

    cartesian_pose_publisher = Node(
        package='wbc_ros',
        executable='cartesian_pose_publisher',
        name='pose',
        namespace='/ee_pose_controller',
        parameters=[pose_publisher_config])

    cartesian_trajectory_publisher = Node(
        package='wbc_ros',
        executable='cartesian_trajectory_publisher',
        name='trajectory',
        namespace='/elbow_pose_controller',
        parameters=[trajectory_publisher_config])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='/',
        output='both',
        parameters=[robot_description])

    return LaunchDescription([
        controller_manager,
        whole_body_controller_spawner,
        delayed_elbow_pose_controller_spawner,
        delayed_ee_pose_controller_spawner,
        cartesian_pose_publisher,
        cartesian_trajectory_publisher,
        joint_state_broadcaster,
        robot_state_publisher
    ])
