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
    # Load the controller confguration file for the example. This contains the parameters for all ros2 controllers. 
    go2_controllers = PathJoinSubstitution([FindPackageShare('wbc_ros'), 'config', 'quadruped_example', 'go2_controllers.yaml'])

    # Create the robot description parameter (URDF) from the iiwa.config.xacro file. Use the "fake" flag, which means that the input command is mirrored to the
    # robot state, producing a simple mini simulation
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),' ',
        PathJoinSubstitution([FindPackageShare('wbc_ros'), 'models', 'urdf', 'unitree', 'go2.config.xacro']),' ',
        'prefix:=',                          '',                       ' ',
        'initial_positions:=',               'initial_positions.yaml', ' ',
        'command_interface:=',               'position',               ' ',
        'namespace:=',                       '/'])
    robot_description = {'robot_description': robot_description}

    # Load the controller manager (ros2_control_node). This is resposible for loading, configuring, activating and deactivating the controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, go2_controllers],
        output='both',
        namespace='/'
    )

    # Spawn the whole-body controller (WBC). The WBC receive the control output from all controllers in task space (in this case only the Cartesian position controller)
    # and integrates into a coherent control signal in joint space (in this case joint position commands), which are written to hardware interface directly
    whole_body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['whole_body_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/')

    # Spawn the Cartesian position controller. This controller stabilizes the desired end effector trajectory in cartesian space
    com_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['com_position_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/')

    # Delay start of cartesian_position_controller_spawner after `whole_body_controller_spawner` to have the reference interfaces in WBC ready
    delayed_com_position_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=whole_body_controller_spawner,
                on_exit=[com_position_controller_spawner])))

    # The joint state broadcaster is actually not really a controller, it simply collects the joint states and publishes them
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', ['/', 'controller_manager']])

    # This node publishes a circular trajectory and sends it to the Cartesian position controller
    #cartesian_trajectory_publisher = Node(
    #    package='wbc_ros',
    #    executable='cartesian_trajectory_publisher',
    #    name='trajectory',
    #    namespace='/cartesian_position_controller',
    #    parameters=[trajectory_publisher_config])

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
        delayed_com_position_controller_spawner,
        #cartesian_trajectory_publisher,
        joint_state_broadcaster,
        robot_state_publisher
    ])
