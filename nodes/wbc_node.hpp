#ifndef WBC_ROS_NODE_HPP
#define WBC_ROS_NODE_HPP

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <wbc/core/RobotModel.hpp>
#include <wbc/core/PluginLoader.hpp>
#include <wbc/core/Scene.hpp>
#include <wbc/core/QPSolver.hpp>
#include <wbc/tools/JointIntegrator.hpp>

#include <base/commands/Joints.hpp>

namespace wbc{

class WbcNode{
protected:
   ros::NodeHandle* nh;
   wbc::WbcScenePtr scene;
   wbc::RobotModelPtr robot_model;
   wbc::QPSolverPtr solver;

   base::samples::Joints joint_state;
   base::samples::RigidBodyStateSE3 reference_cart;
   base::commands::Joints reference_jnt;
   base::VectorXd task_weights;
   wbc::JointWeights joint_weights;
   bool is_initialized;
   bool integrate;
   double control_rate;
   HierarchicalQP qp;
   base::commands::Joints solver_output;
   wbc::JointIntegrator joint_integrator;

   trajectory_msgs::JointTrajectory solver_output_ros;

   std::vector<ros::Subscriber> subscribers;
   ros::Subscriber sub_joint_state;
   ros::Publisher solver_output_publisher;

   void jointStateCallback(const sensor_msgs::JointState& msg);
   void cartReferenceCallback(const ros::MessageEvent<geometry_msgs::TwistStamped>& event, const std::string& constraint_name);
   void jntReferenceCallback(const ros::MessageEvent<trajectory_msgs::JointTrajectory>& event, const std::string& constraint_name);
   void taskActivationCallback(const ros::MessageEvent<std_msgs::Float64>& event, const std::string& constraint_name);
   void taskWeightsCallback(const ros::MessageEvent<std_msgs::Float64MultiArray>& event, const std::string& constraint_name);
   void jointWeightsCallback(const std_msgs::Float64MultiArray& msg);

public:
   WbcNode(int argc, char** argv);
   ~WbcNode();
   void solve();
   bool isInitialized(){return is_initialized;}
   double controlRate(){return control_rate;}
};

}

#endif
