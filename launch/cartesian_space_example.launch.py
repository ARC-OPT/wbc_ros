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
    # Load configuration files
    base_path = os.path.join(get_package_share_directory('wbc_ros'),'config','cartesian_space_example')
    wbc_config = base_path + '/whole_body_controller.yaml'
    controller_config = base_path + '/cartesian_position_controller.yaml'
    trajectory_publisher_config = base_path + '/trajectory_publisher.yaml'
    ros2_controllers = base_path + '/ros2_controllers.yaml'

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
        parameters=[robot_description, ros2_controllers],
        output='both',
        namespace='/'
    )

    # Spawn position forward controller
    position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/')
    
    # The joint state broadcaster is actually not really a controller, it simply collects the joint states and publishes them
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', ['/', 'controller_manager']])
    
    # The robot state publisher computes the transform between all robot links given the joint_status topic to visualize the robot in rviz
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='/',
        output='both',
        parameters=[robot_description])
     
    # This node publishes a circular trajectory and sends it to the Cartesian position controller
    cartesian_trajectory_publisher = Node(
        package='wbc_ros',
        executable='cartesian_trajectory_publisher',
        name='trajectory',
        namespace='/cartesian_position_controller',
        parameters=[trajectory_publisher_config])
     
    # Spawn the whole-body controller (WBC). The WBC receives the control output from all controllers in task space (in this case only the Cartesian position controller)
    # and integrates into a coherent control signal in joint space (in this case joint position commands), which are written to hardware interface directly
    whole_body_controller = Node(
        package='wbc_ros',
        executable='whole_body_controller_node',
        name='whole_body_controller',
        parameters=[robot_description, wbc_config])

    # Spawn a Cartesian position controller
    cartesian_position_controller = Node(
        package='wbc_ros',
        executable='cartesian_position_controller_node',
        name='cartesian_position_controller',
        parameters=[controller_config])
    
    return LaunchDescription([
        controller_manager,
        position_controller,
        joint_state_broadcaster,
        robot_state_publisher,
        whole_body_controller,
        cartesian_position_controller,
        cartesian_trajectory_publisher
    ])
