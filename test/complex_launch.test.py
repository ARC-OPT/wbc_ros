import time
import unittest
import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

@pytest.mark.launch_test
def generate_test_description():

    prefix = get_package_share_directory('wbc_ros')
    default_args = ['--ros-args', '--log-level', 'fatal']

    return launch.LaunchDescription([
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
		             package='wbc_ros', name='wbc', namespace='wbc', executable='wbc',
		             parameters=[prefix + '/test/config/wbc_multi_tasks.yml', {'robot_model_config.file': prefix + '/models/urdf/kuka_iiwa.urdf'}],
                     arguments=['--ros-args', '--log-level', 'fatal']
                ),
                launch_ros.actions.Node(
		             package='wbc_ros', name='controller', namespace='wbc/ee_pose', executable='cartesian_position_controller',
		             parameters=[prefix + '/test/config/ee_pose_controller.yml'],
                     arguments=default_args
                ),
                launch_ros.actions.Node(
		             package='wbc_ros', name='controller', namespace='wbc/collision_avoidance', executable='cartesian_radial_potential_fields',
		             parameters=[prefix + '/test/config/collision_avoidance_controller.yml'],
                     arguments=default_args
                ),
                launch_ros.actions.Node(
		             package='wbc_ros', name='controller',namespace='wbc/contact_force',executable='cartesian_force_controller',
		             parameters=[prefix + '/test/config/contact_force_controller.yml'],
                     arguments=default_args
                ),
                launch_ros.actions.Node(
		             package='wbc_ros', name='controller', namespace='wbc/joint_position', executable='joint_position_controller',
		             parameters=[prefix + '/test/config/joint_position_controller.yml'],
                     arguments=default_args
                ),
                launch_ros.actions.Node(
		             package='wbc_ros', name='controller', namespace='wbc/joint_limits', executable='joint_limit_avoidance',
		             parameters=[prefix + '/test/config/joint_limits_controller.yml'],
                     arguments=default_args
                ),
                launch_ros.actions.Node(
		             package='wbc_ros', name='joints', executable='loop_back_driver', namespace='wbc',
		             parameters=[prefix + '/test/config/joints.yml'],
                     arguments=default_args
                )
            ]),
        launch_testing.actions.ReadyToTest()
     ])


class TestComplexWBCLaunch(unittest.TestCase):
    def test_state_feedback(self):
        rclpy.init()
        try:
            node = TestNode()
            state = node.read_state('/wbc/state')
            assert state == 'RUNNING' or state == 'NO_FEEDBACK', 'State of WBC node is invalid!'
            task_names = ['ee_pose', 'collision_avoidance', 'contact_force', 'joint_position', 'joint_limits']
            for task_name in task_names:
                state = node.read_state('/wbc/' + task_name + '/state')
                assert state == 'RUNNING' or state == 'NO_FEEDBACK', 'State of ' + task_name + ' is invalid!'
                time.sleep(2)
        finally:
            rclpy.shutdown()

class TestNode(Node):
    def __init__(self, name='wbc_test_node'):
        super().__init__(name)

    def state_callback(self, msg):
        self.state = msg.data

    def read_state(self, state_topic, timeout=10.0):
        self.subscription = self.create_subscription(String, state_topic, self.state_callback, 10)
        self.state = None
        start = time.time()
        while time.time() - start < timeout and self.state == None:
            rclpy.spin_once(self)
        return self.state
