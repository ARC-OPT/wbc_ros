#ifndef WBC_ROS_NODE_HPP
#define WBC_ROS_NODE_HPP

#include <wbc_msgs/RigidBodyState.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <wbc_msgs/TaskStatus.h>
#include <wbc_msgs/WbcTimingStats.h>

#include "controllers/controller_node.hpp"
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/PluginLoader.hpp>
#include <wbc/core/Scene.hpp>
#include <wbc/core/QPSolver.hpp>
#include <wbc/tools/JointIntegrator.hpp>

#include <base/commands/Joints.hpp>

/**
@brief WBCNode - Main ROS interface for the WBC libary (https://github.com/ARC-OPT/wbc). This node implements a peridic control loop,
which does the following in every control cycle:

   1. Update the internal robot with the current joint status
   2. Update the reference values for all tasks
   3. Set up a quadratic program, which includes all tasks and constraints
   4. Solve the QP and outputs the solver output
   5. Publish debug info about the task status and the QP

Subscribed Topics:
 - `joint_state` (`sensor_msgs/JointState`): The current joint state of the entire robot. Must contain all non-fixed joints from the URDF used in WBC. Must contain
    at least positions for Veloicty-based WBC and positions/velocities for acceleration-based WBC, e.g., TSID
 - `floating_base_state` (`wbc_msgs/RigidBodyState`): The state of the floating base. Only required if floating_base is set to true in the wbc configuration. Must contain
    at least pose for Veloicty-based WBC and pose/twist for acceleration-based WBC, e.g., TSID
 - `ref_mytask` (`wbc_msgs/RigidBodyState` or `sensor_msgs/JointState`): Reference for task mytask. For Cartesian space tasks this must be a twist or spatial acceleration,
    for joint space tasks, this must be joint velocities or accelerations.
 - `weights_mytask` (`std_msgs/Float64MultiArray`): Task weights for task mytask. Can be used to set the priority of different aspects of one tasks, e.g.,
    certain directions in task space. Size must be 6 for Cartesian space tasks. For joint space tasks, the size must match the number of configured joints for this task.
 - `activation_mytask` (`std_msgs/Float64`): Activation value for task mytask. Can be used to activate/deactivate a task entirely.
 - `joint_weights` (`std_msgs/Float64MultiArray`): Joint weight vector. Can be used to prioritize or activate/deactivate certain joints.

 Published Topics:
 - `status_mytask` (`wbc_msgs/RigidBodyState` or `sensor_msgs/JointState`): Status for task mytask For Cartesian space tasks: Current pose, twist and
   spatial acceleration for the kinematic chain associated with this task. For joint space tasks: position, velocity and acceleration of the joints associated with this task.
 - `task_mytask` (`wbc_msgs/TaskStatus`): Debug info on task mytask.
 - `solver_output` (`trajectory_msgs/JointTrajectory`): Solution of the QP. No. of points in the trajectory is always 1. Size of this point is same as number
    of non-fixed joints in URDF.
 - `timing_stats` (`wbc_msgs/WbcTimingStats`): Computation time for different processing steps in WBC.

 Parameters:
 - `control_rate` (double): Loop rate of the WBC in Hz
 - `integrate` (bool): If true, the solver output will be integrated once for velocity-based WBC and twice for acceleration-based WBC
 - `robot_model_config` (dict): Configuration of the robot model like, e.g., URDF filename, type of model, floating base etc.
    (see <a href="https://github.com/ARC-OPT/wbc/blob/master/src/core/RobotModelConfig.hpp">here</a>) for more information.
 - `qp_solver` (string): Name of the QP solver to use, can be one of qpoases, qpswift, eiquadprog, proxqp
 - `wbc_config` (dict): Tasks configuration. This contains information about each task, which is stacked inside the QP. See
   (see <a href="https://github.com/ARC-OPT/wbc/blob/master/src/core/TaskConfig.hpp">here</a>) for more information.
*/
class WbcNode : public ControllerNode{
protected:
   wbc::WbcScenePtr scene;
   wbc::RobotModelPtr robot_model;
   wbc::QPSolverPtr solver;

   base::samples::Joints joint_state;
   base::samples::RigidBodyStateSE3 reference_cart;
   base::samples::RigidBodyStateSE3 floating_base_state;
   base::commands::Joints reference_jnt;
   base::VectorXd task_weights;
   wbc::JointWeights joint_weights;
   bool integrate;
   wbc::HierarchicalQP qp;
   base::commands::Joints solver_output;
   wbc::JointIntegrator joint_integrator;
   std::vector<wbc::TaskConfig> wbc_config;
   bool has_floating_base_state;
   wbc::TasksStatus tasks_status;
   std::vector<wbc_msgs::TaskStatus> task_status_msgs;

   trajectory_msgs::JointTrajectory solver_output_ros;

   std::vector<ros::Subscriber> subscribers;
   std::vector<ros::Publisher> publishers_task_status;
   std::vector<ros::Publisher> publishers_task_info;
   ros::Subscriber sub_joint_weights;
   ros::Subscriber sub_floating_base;
   ros::Publisher solver_output_publisher;
   ros::Publisher pub_timing_stats;
   wbc_msgs::RigidBodyState status_cart;
   sensor_msgs::JointState status_jnt;
   wbc_msgs::WbcTimingStats timing_stats;
   ros::Time stamp;

   void jointStateCallback(const sensor_msgs::JointState& msg);
   void cartReferenceCallback(const ros::MessageEvent<wbc_msgs::RigidBodyState>& event, const std::string& constraint_name);
   void jntReferenceCallback(const ros::MessageEvent<trajectory_msgs::JointTrajectory>& event, const std::string& constraint_name);
   void taskActivationCallback(const ros::MessageEvent<std_msgs::Float64>& event, const std::string& constraint_name);
   void taskWeightsCallback(const ros::MessageEvent<std_msgs::Float64MultiArray>& event, const std::string& constraint_name);
   void jointWeightsCallback(const std_msgs::Float64MultiArray& msg);
   void floatingBaseStateCallback(const wbc_msgs::RigidBodyState& msg);

public:
   WbcNode(int argc, char** argv);
   ~WbcNode();
   virtual void updateController();
   void publishTaskStatus();
   void publishTaskInfo();
};

#endif
