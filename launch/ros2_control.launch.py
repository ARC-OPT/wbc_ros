from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,Command,FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

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

    whole_body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['whole_body_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/',
        remappings=[("/whole_body_controller/status_ee_pose","/cartesian_position_controller/feedback")]
    )
    cartesian_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_position_controller', '--controller-manager', ['/', 'controller_manager']],
        namespace='/'
    )

    # Delay start of forward_position_controller_spawner after `position_controller_spawner`
    delay_cartesian_position_controller_spawner_after_whole_body_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=whole_body_controller_spawner,
                on_exit=[cartesian_position_controller_spawner],
            )
        )
    )
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, iiwa_controllers],
            output='both',
            namespace='/'
        ),
        whole_body_controller_spawner,
        delay_cartesian_position_controller_spawner_after_whole_body_controller_spawner,
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
