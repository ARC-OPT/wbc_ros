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
    robot_urdf    = prefix + '/models/urdf/kuka_iiwa.urdf'
    wbc_config    = prefix + '/test/config/wbc_single_cartesian_task.yml'
    return launch.LaunchDescription([
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
		             package='wbc_ros', name='wbc', namespace='wbc', executable='wbc',
		             parameters=[wbc_config, {'robot_model_config.file': robot_urdf}],
                     arguments=['--ros-args', '--log-level', 'fatal']
                ),
                launch_ros.actions.Node(
		             package='wbc_ros', name='joints', executable='loop_back_driver', namespace='wbc',
		             parameters=[prefix + '/test/config/joints.yml'],
                     arguments=['--ros-args', '--log-level', 'fatal']
                )
            ]),
        launch_testing.actions.ReadyToTest()
     ])

class TestNode(Node):
    def __init__(self, name='wbc_test_node'):
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            if node_name in self.get_node_names():
                return True
        return False

class TestBringup(unittest.TestCase):
    def test_wbc_node_start(self):
        rclpy.init()
        try:
            node = TestNode()
            assert node.wait_for_node('wbc'), 'WBC Node not found !'
            time.sleep(1)
        finally:
            rclpy.shutdown()
