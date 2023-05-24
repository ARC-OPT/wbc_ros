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

@pytest.mark.launch_test
def generate_test_description():

    prefix = get_package_share_directory('wbc_ros')

    return launch.LaunchDescription([
     launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
		             package='wbc_ros', name='wbc', executable='wbc', namespace='wbc',
		             parameters=[prefix + '/test/config/wbc_single_cartesian_task.yml', {'robot_model_config.file': prefix + '/models/urdf/kuka_iiwa.urdf'}],
                     arguments=['--ros-args', '--log-level', 'fatal']
                )
            ]),
        launch_testing.actions.ReadyToTest()
     ])

class TestTopicsExist(unittest.TestCase):
    def test_publishers_subscribers(self):
        rclpy.init()
        try:
            receiver = TestNode()
            assert receiver.wait_for_node('wbc'), 'WBC node not found !'
            time.sleep(1)
            task_name = 'ee_pose'
            assert receiver.has_publisher('/task_' + task_name)
            assert receiver.has_publisher('/status_' + task_name)
            assert receiver.has_subscriber('/activation_' + task_name)
            assert receiver.has_subscriber('/weights_' + task_name)
            assert receiver.has_subscriber('/ref_' + task_name)
        finally:
            rclpy.shutdown()

class TestNode(Node):
    def __init__(self, name='test_node'):
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            if node_name in self.get_node_names():
                return True
        return False

    def has_publisher(self,name):
        publishers = self.get_publisher_names_and_types_by_node('wbc','/wbc')
        publishers_names = [name for name, type in publishers]
        return any(name in mystring for mystring in publishers_names)

    def has_subscriber(self,name):
        subscribers = self.get_subscriber_names_and_types_by_node('wbc','/wbc')
        subscriber_names = [name for name, type in subscribers]
        return any(name in mystring for mystring in subscriber_names)
