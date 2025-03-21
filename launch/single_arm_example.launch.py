from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,Command,FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os

def generate_launch_description():
    # Load configuration files
    base_path = os.path.join(get_package_share_directory('wbc_ros'),'config','single_arm_example')
    wbc_config = base_path + '/whole_body_controller.yaml'
    mock_hardware_config = base_path + '/mock_hardware_interface.yaml'
    trajectory_publisher_config = base_path + '/trajectory_publisher.yaml'

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
        name='cartesian_trajectory_publisher',
        namespace='',
        remappings=[('/setpoint', '/whole_body_controller/ee_pose/setpoint')],
        parameters=[trajectory_publisher_config])

    # WBC and mock hardware interface container
    container = ComposableNodeContainer(
        name='wbc_node',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='wbc_ros',
                plugin='wbc_ros::SingleArmController',
                name='whole_body_controller',
                remappings=[('/whole_body_controller/solver_output', '/mock_hardware_interface/command')],
                parameters=[robot_description, wbc_config]),
            ComposableNode(
                package='wbc_ros',
                plugin='wbc_ros::MockHardwareInterface',
                name='mock_hardware_interface',
                namespace='',
                remappings=[('/mock_hardware_interface/robot_state', '/whole_body_controller/robot_state')],
                parameters=[mock_hardware_config])
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        container,
        cartesian_trajectory_publisher
    ])
