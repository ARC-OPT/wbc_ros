import time
import unittest
import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
from launch.substitutions import PathJoinSubstitution,Command,FindExecutable
from launch_ros.substitutions import FindPackageShare
import pytest
import rclpy
from rclpy.node import Node

@pytest.mark.launch_test
def generate_test_description():
    robot_controllers = PathJoinSubstitution([FindPackageShare('wbc_ros'), 'test', 'config', 'robot_controllers_simple.yaml'])
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

    return launch.LaunchDescription([
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
		             package='controller_manager', namespace='/', executable='ros2_control_node',
		             parameters=[robot_description,robot_controllers],
                     arguments=['--ros-args', '--log-level', 'fatal']
                ),
                launch_ros.actions.Node(
		             package='controller_manager', namespace='/', executable='spawner',
                     arguments=['whole_body_controller', '--controller-manager', ['/', 'controller_manager']],
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
            assert node.wait_for_node('whole_body_controller'), 'WBC Node not found !'
            time.sleep(1)
        finally:
            rclpy.shutdown()
