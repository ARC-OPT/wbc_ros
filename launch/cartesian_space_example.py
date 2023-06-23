import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    wbc_config          = os.path.join(get_package_share_directory('wbc_ros'),'config','cartesian_space_example','wbc.yml')
    joints_config       = os.path.join(get_package_share_directory('wbc_ros'),'config','joints.yml')
    ee_pose_ctrl_config = os.path.join(get_package_share_directory('wbc_ros'),'config','cartesian_space_example','ee_pose_controller.yml')
    ee_pose_traj_config = os.path.join(get_package_share_directory('wbc_ros'),'config','cartesian_space_example','ee_pose_trajectory.yml')
    urdf_file           = os.path.join(get_package_share_directory('wbc_ros'),'models','urdf','kuka_iiwa.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    task_name = "ee_pose"
    return LaunchDescription([
        PushRosNamespace('kuka_iiwa'),
        Node(
            package='wbc_ros',
            namespace='wbc',
            executable='wbc',
            name='wbc',
            parameters=[wbc_config,{'robot_model_config.file': urdf_file}],
            remappings=[("/kuka_iiwa/wbc/solver_output","/kuka_iiwa/command"),
                        ("/kuka_iiwa/wbc/joint_states","/kuka_iiwa/joint_states"),
                        ("/kuka_iiwa/wbc/ref_"+task_name,"/kuka_iiwa/wbc/"+task_name+"/control_output"),
                        ("/kuka_iiwa/wbc/status_"+task_name,"/kuka_iiwa/wbc/"+task_name+"/feedback")]
        ),
        Node(
            package='wbc_ros',
            namespace='wbc/'+task_name,
            executable='cartesian_position_controller',
            name='controller',
            parameters=[ee_pose_ctrl_config]
        ),
        Node(
            package='wbc_ros',
            namespace='wbc/'+task_name,
            executable='cartesian_trajectory_publisher',
            name='trajectory',
            parameters=[ee_pose_traj_config]
        ),
        Node(
            package='wbc_ros',
            namespace='',
            executable='loop_back_driver',
            name='joints',
            parameters=[joints_config]
        ),
        Node(
            package='robot_state_publisher',
            namespace='',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{"robot_description": robot_desc}]
        )
    ])
