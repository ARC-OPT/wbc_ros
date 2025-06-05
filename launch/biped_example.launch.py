from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,Command,FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
import xacro

def generate_launch_description():
    # Load configuration files
    base_path = os.path.join(get_package_share_directory('wbc_ros'),'config','biped_example')
    wbc_config = base_path + '/whole_body_controller.yaml'
    mock_hardware_config = base_path + '/mock_hardware_interface.yaml'
    trajectory_publisher_config_l = base_path + '/trajectory_publisher_foot_l.yaml'
    trajectory_publisher_config_r = base_path + '/trajectory_publisher_foot_r.yaml'
    trajectory_publisher_config = base_path + '/trajectory_publisher_joint.yaml'
    urdf_path = os.path.join(get_package_share_directory('wbc_ros'), 'models', 'urdf', 'hyper', 'HyPer-1.urdf')
    
    # Create the robot description parameter (URDF) from the iiwa.config.xacro file. Use the "fake" flag, which means that the input command is mirrored to the
    # robot state, producing a simple mini simulation
    with open(urdf_path, 'r') as file:
        robot_description = file.read()
    robot_description = {'robot_description': robot_description}

    # The robot state publisher computes the transform between all robot links given the joint_status topic to visualize the robot in rviz
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='/',
        output='both',
        parameters=[robot_description])

    # This node publishes a circular trajectory and sends it to the Cartesian position controller
    trajectory_publisher_l = Node(
        package='wbc_ros',
        executable='cartesian_trajectory_publisher',
        name='foot_l_trajectory_publisher',
        namespace='',
        remappings=[('/setpoint', '/whole_body_controller/foot_l_pose/setpoint')],
        parameters=[trajectory_publisher_config_l])
    
    trajectory_publisher_r = Node(
        package='wbc_ros',
        executable='cartesian_trajectory_publisher',
        name='foot_r_trajectory_publisher',
        namespace='',
        remappings=[('/setpoint', '/whole_body_controller/foot_r_pose/setpoint')],
        parameters=[trajectory_publisher_config_r])
    
    # This node publishes a circular trajectory and sends it to the Cartesian position controller
    joint_trajectory_publisher = Node(
        package='wbc_ros',
        executable='joint_trajectory_publisher',
        name='joint_trajectory_publisher',
        namespace='',
        remappings=[('/setpoint', '/whole_body_controller/joint_position/setpoint')],
        parameters=[trajectory_publisher_config])
      
    # Convert robot state to joint states
    converter = Node(
        package='wbc_ros',
        executable='joint_state_converter',
        name='joint_state_converter',
        namespace='',
        remappings=[('/joint_state_converter/joint_state', '/cubemars_hardware_node/joint_states')],
        parameters=[{'joint_names': ["joint_ll_hip_1","joint_ll_hip_2","joint_ll_knee",
                                     "joint_rl_hip_1","joint_rl_hip_2","joint_rl_knee"]}])
        
    # WBC and mock hardware interface container
    container = ComposableNodeContainer(
        name='wbc_node',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='wbc_ros',
                plugin='wbc_ros::BipedController',
                name='whole_body_controller',
                #remappings=[('/whole_body_controller/solver_output', '/cubemars_hardware_node/joint_commands'),
                #            ('/whole_body_controller/joint_state', '/cubemars_hardware_node/joint_states')],
                remappings=[('/whole_body_controller/solver_output', '/mock_hardware_interface/command'),
                            ('/whole_body_controller/joint_state', '/mock_hardware_interface/robot_state')],
                parameters=[robot_description, wbc_config]),
            ComposableNode(
                package='wbc_ros',
                plugin='wbc_ros::MockHardwareInterface',
                name='mock_hardware_interface',
                namespace='',
                parameters=[mock_hardware_config])
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        converter,
        joint_trajectory_publisher,
        #trajectory_publisher_l,
        #trajectory_publisher_r,   
        container
    ])
