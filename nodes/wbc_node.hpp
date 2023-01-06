#ifndef WBC_ROS_NODE_HPP
#define WBC_ROS_NODE_HPP

#include <wbc_msgs/RigidBodyState.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <wbc_msgs/TaskStatus.h>

#include "controllers/controller_node.hpp"
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/PluginLoader.hpp>
#include <wbc/core/Scene.hpp>
#include <wbc/core/QPSolver.hpp>
#include <wbc/tools/JointIntegrator.hpp>

#include <base/commands/Joints.hpp>

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
   wbc_msgs::RigidBodyState status_cart;
   sensor_msgs::JointState status_jnt;

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
