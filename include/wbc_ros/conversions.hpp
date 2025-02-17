#ifndef WBC_ROS_CONVERSIONS_HPP
#define WBC_ROS_CONVERSIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wbc_msgs/msg/rigid_body_state.hpp>
#include <wbc_msgs/msg/task_status.hpp>
#include <wbc_msgs/msg/joint_command.hpp>
#include <wbc_msgs/msg/joint_state.hpp>
#include <wbc_msgs/msg/robot_state.hpp>
#include <wbc_msgs/msg/contacts.hpp>

#include <wbc/types/JointState.hpp>
#include <wbc/types/JointCommand.hpp>
#include <wbc/types/RigidBodyState.hpp>
#include <wbc/types/Wrench.hpp>
#include <wbc/types/Contact.hpp>

void fromROS(const geometry_msgs::msg::Pose& in, wbc::types::Pose& out);
void fromROS(const geometry_msgs::msg::Twist& in, wbc::types::Twist& out);
void fromROS(const geometry_msgs::msg::Accel& in, wbc::types::SpatialAcceleration& out);
void fromROS(const geometry_msgs::msg::Wrench& in, wbc::types::Wrench& out);

void fromROS(const wbc_msgs::msg::JointState& in, wbc::types::JointState& out);
void fromROS(const wbc_msgs::msg::RigidBodyState& in, wbc::types::RigidBodyState& out);
void fromROS(const wbc_msgs::msg::JointCommand& in, wbc::types::JointCommand& out);
void fromROS(const wbc_msgs::msg::Contacts& in, std::vector<wbc::types::Contact>& out);
void fromROS(const std_msgs::msg::Float64MultiArray& in, Eigen::VectorXd& out);
void fromROS(const wbc_msgs::msg::RobotState& in, wbc::types::JointState &joint_state_out, wbc::types::RigidBodyState& floating_base_state_out);

void toROS(const wbc::types::RigidBodyState& in, wbc_msgs::msg::RigidBodyState& out);
void toROS(const wbc::types::JointCommand& in, wbc_msgs::msg::JointCommand& out);
void toROS(const Eigen::VectorXd& in, std_msgs::msg::Float64MultiArray& out);
void toROS(const wbc::types::Pose& in, geometry_msgs::msg::Pose& out);
void toROS(const wbc::types::Twist& in, geometry_msgs::msg::Twist& out);
void toROS(const wbc::types::SpatialAcceleration& in, geometry_msgs::msg::Accel& out);
void toROS(const wbc::types::Pose& pose, const wbc::types::Twist& twist, const wbc::types::SpatialAcceleration& acc, wbc_msgs::msg::RigidBodyState& out);
void toROS(const wbc::types::JointState& in, wbc_msgs::msg::JointState& out);

#endif
