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
#include <wbc/core/RobotModelFactory.hpp>
#include <wbc/core/Scene.hpp>
#include <wbc/core/QPSolver.hpp>

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
   HierarchicalQP qp;
   base::commands::Joints solver_output;

   trajectory_msgs::JointTrajectory solver_output_ros;

   std::vector<ros::Subscriber> subscribers;
   ros::Publisher solver_output_publisher;

   base::RigidBodyStateSE3 xmlrpc2RigidBodyStateSE3(const XmlRpc::XmlRpcValue& in);
   wbc::RobotModelConfig xmlrpc2RobotModelConfig(const XmlRpc::XmlRpcValue& in);
   std::vector<wbc::ConstraintConfig> xmlrpc2WbcConfig(const XmlRpc::XmlRpcValue& in);

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
};

}

#endif
