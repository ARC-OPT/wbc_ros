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
from trajectory_msgs.msg import JointTrajectory

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
                ),
                launch_ros.actions.Node(
		             package='wbc_ros', name='joints', executable='loop_back_driver', namespace='wbc',
		             parameters=[prefix + '/test/config/joints.yml'],
                     arguments=['--ros-args', '--log-level', 'fatal']
                )
            ]),
        launch_testing.actions.ReadyToTest()
     ])

class TestSolverOutput(unittest.TestCase):
    def test_solver_output(self):
        rclpy.init()
        try:
            receiver = TestNode()
            solver_output = receiver.read_solver_output()
            joint_names = ['joint_0','joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
            assert solver_output.joint_names == joint_names, 'Invalid joint names in Solver output'
            assert solver_output.points[0].velocities.tolist() == [0]*7, 'Invalid output velocities in Solver output'
        finally:
            rclpy.shutdown()

class TestNode(Node):
    def __init__(self, name='wbc_test_node'):
        super().__init__(name)

    def solver_output_callback(self, msg):
        self.solver_output = msg

    def read_solver_output(self, timeout=8.0):
        start = time.time()
        self.subscription = self.create_subscription(JointTrajectory, '/wbc/solver_output', self.solver_output_callback, 10)
        self.solver_output = None
        while time.time() - start < timeout and self.solver_output == None:
            rclpy.spin_once(self)
        return self.solver_output
