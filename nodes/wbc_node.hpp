#ifndef WBC_ROS_NODE_HPP
#define WBC_ROS_NODE_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <wbc/core/RobotModel.hpp>
#include <wbc/core/PluginLoader.hpp>
#include <wbc/core/RobotModelFactory.hpp>
#include <wbc/core/Scene.hpp>
#include <wbc/core/QPSolver.hpp>

#include <base/samples/Joints.hpp>

class WbcNode{
protected:
   ros::NodeHandle* nh;
   wbc::WbcScenePtr scene;
   wbc::RobotModelPtr robot_model;
   wbc::QPSolverPtr solver;
   
   base::samples::Joints joint_state;
   
   ros::Subscriber joint_state_suscriber;
   
   base::RigidBodyStateSE3 xmlrpc2RigidBodyStateSE3(const XmlRpc::XmlRpcValue& in);
   wbc::RobotModelConfig xmlrpc2RobotModelConfig(const XmlRpc::XmlRpcValue& in);
   std::vector<wbc::ConstraintConfig> xmlrpc2WbcConfig(const XmlRpc::XmlRpcValue& in);
   
   void jointStateCallback(const sensor_msgs::JointState& msg);
   
public:
   WbcNode(int argc, char** argv);
   ~WbcNode();
   void update();
};

#endif
