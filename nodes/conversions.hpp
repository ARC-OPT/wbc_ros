#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include <XmlRpcValue.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>

#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/commands/Joints.hpp>
#include <wbc/core/TaskConfig.hpp>
#include <wbc/core/RobotModelConfig.hpp>
#include <wbc/core/QuadraticProgram.hpp>

void fromROS(const XmlRpc::XmlRpcValue& in, base::RigidBodyStateSE3& out);
void fromROS(const XmlRpc::XmlRpcValue& in, wbc::RobotModelConfig& out);
void fromROS(const XmlRpc::XmlRpcValue& in, wbc::ActiveContact& out);
void fromROS(const XmlRpc::XmlRpcValue& in, std::vector<wbc::TaskConfig>& out);
void fromROS(const sensor_msgs::JointState& in, base::samples::Joints& out);
void fromROS(const geometry_msgs::TwistStamped& in, base::samples::RigidBodyStateSE3& out);
void fromROS(const trajectory_msgs::JointTrajectory& in, base::commands::Joints& out);
void fromROS(const std_msgs::Float64MultiArray& in, base::VectorXd& out);
void fromROS(const std_msgs::Float64MultiArray& in, const std::vector<std::string>& joint_names, wbc::JointWeights& out);
void toROS(const base::commands::Joints& in, trajectory_msgs::JointTrajectory& out);

#endif
