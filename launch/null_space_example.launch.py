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
    # Load the controller confguration file for the joint space example. This contains the parameters for all ros2 controllers.
    iiwa_controllers = PathJoinSubstitution([FindPackageShare('wbc_ros'), 'config', 'null_space_example', 'iiwa_controllers.yaml'])
    # Load the trajectory configuration file
    trajectory_publisher_config = os.path.join(get_package_share_directory('wbc_ros'),'config','null_space_example','trajectory_publisher.yml')
    # Load the pose configuration file
    pose_publisher_config = os.path.join(get_package_share_directory('wbc_ros'),'config','null_space_example','pose_publisher.yml')

    # Create the robot description parameter (URDF) from the iiwa.config.xacro file. Use the "fake" flag, which means that the input command is mirrored to the
    # robot state, producing a simple mini simulation
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),' ',
        PathJoinSubstitution([FindPackageShare('wbc_ros'), 'models', 'urdf', 'kuka', 'iiwa.config.xacro']),' ',
        'prefix:=',                          '',                       ' ',
        'use_sim:=',                         'false',                  ' ',
        'use_fake_hardware:=',               'true',                   ' ',
        'initial_positions:=',               'initial_positions.yaml', ' ',
        'command_interface:=',               'position',               ' ',
        'namespace:=',                       '/'])
    robot_description = {'robot_description': robot_description}

    # Load the controller manager (ros2_control_node). This is resposible for loading, configuring, activating and deactivating the controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, iiwa_controllers],
        output='both',
        namespace='/'
    )

    # Spawn the whole-body controller (WBC). The WBC receive the control output from all controllers in task space (in this case only the joint position controller)
    # and integrates into a coherent control signal in joint space (in this case joint position commands), which are written to hardware interface directly
    whole_body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['whole_body_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/')

    # Spawn the ee controller. This controller stabilizes the desired end effector pose in cartesian space
    ee_pose_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ee_pose_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/')

    # Spawn the elbow controller. This controller stabilizes the desired elbow trajectory in cartesian space
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

    # The joint state broadcaster is actually not really a controller, it simply collects the joint states and publishes them
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', ['/', 'controller_manager']])

    # This node publishes a fixed pose and sends it to the ee controller
    cartesian_pose_publisher = Node(
        package='wbc_ros',
        executable='cartesian_pose_publisher',
        name='pose',
        namespace='/ee_pose_controller',
        parameters=[pose_publisher_config])

    # This node publishes a circular trajectory and sends it to the elbow controller
    cartesian_trajectory_publisher = Node(
        package='wbc_ros',
        executable='cartesian_trajectory_publisher',
        name='trajectory',
        namespace='/elbow_pose_controller',
        parameters=[trajectory_publisher_config])

    # The robot state publisher computes the transform between all robot links given the joint_status topic to visualize the robot in rviz
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
