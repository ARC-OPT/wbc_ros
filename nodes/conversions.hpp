#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include <XmlRpcValue.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <wbc_ros/RigidBodyState.h>

#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/commands/Joints.hpp>
#include <wbc/core/TaskConfig.hpp>
#include <wbc/core/RobotModelConfig.hpp>
#include <wbc/core/QuadraticProgram.hpp>

void fromROS(const XmlRpc::XmlRpcValue& in, base::RigidBodyStateSE3& out);
void fromROS(const XmlRpc::XmlRpcValue& in, wbc::RobotModelConfig& out);
void fromROS(const XmlRpc::XmlRpcValue& in, wbc::ActiveContact& out);
void fromROS(const XmlRpc::XmlRpcValue& in, std::vector<wbc::TaskConfig>& out);
void fromROS(const XmlRpc::XmlRpcValue& in, std::vector<std::string>& out);
void fromROS(const sensor_msgs::JointState& in, base::samples::Joints& out);
void fromROS(const geometry_msgs::Pose& in, base::Pose& out);
void fromROS(const geometry_msgs::Twist& in, base::Twist& out);
void fromROS(const geometry_msgs::Accel& in, base::Acceleration& out);
void fromROS(const wbc_ros::RigidBodyState& in, base::samples::RigidBodyStateSE3& out);
void fromROS(const trajectory_msgs::JointTrajectory& in, base::samples::Joints& out);
void fromROS(const std_msgs::Float64MultiArray& in, base::VectorXd& out);
void fromROS(const std_msgs::Float64MultiArray& in, const std::vector<std::string>& joint_names, wbc::JointWeights& out);
void toROS(const base::commands::Joints& in, trajectory_msgs::JointTrajectory& out);
void toROS(const base::Pose& in, geometry_msgs::Pose& out);
void toROS(const base::Twist& in, geometry_msgs::Twist& out);
void toROS(const base::Acceleration& in, geometry_msgs::Accel& out);
void toROS(const base::samples::RigidBodyStateSE3& in, wbc_ros::RigidBodyState& out);
void toROS(const base::samples::Joints& in, sensor_msgs::JointState& out);

#endif
