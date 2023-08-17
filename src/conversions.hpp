#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wbc_msgs/msg/rigid_body_state.hpp>
#include <wbc_msgs/msg/task_config.hpp>
#include <wbc_msgs/msg/task_status.hpp>
#include <wbc_msgs/msg/radial_potential_field_vector.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/commands/Joints.hpp>
#include <base/samples/Wrench.hpp>
#include <base/JointLimits.hpp>
#include <wbc/core/TaskConfig.hpp>
#include <wbc/core/TaskStatus.hpp>
#include <wbc/core/RobotModelConfig.hpp>
#include <wbc/core/QuadraticProgram.hpp>

void fromROS(const sensor_msgs::msg::JointState& in, base::samples::Joints& out);
void fromROS(const geometry_msgs::msg::Pose& in, base::Pose& out);
void fromROS(const geometry_msgs::msg::Twist& in, base::Twist& out);
void fromROS(const geometry_msgs::msg::Accel& in, base::Acceleration& out);
void fromROS(const wbc_msgs::msg::RigidBodyState& in, base::samples::RigidBodyStateSE3& out);
void fromROS(const trajectory_msgs::msg::JointTrajectory& in, base::samples::Joints& out);
void fromROS(const std_msgs::msg::Float64MultiArray& in, base::VectorXd& out);
void fromROS(const std_msgs::msg::Float64MultiArray& in, const std::vector<std::string>& joint_names, wbc::JointWeights& out);
void fromROS(const geometry_msgs::msg::WrenchStamped& in, base::samples::Wrench& out);
void toROS(const base::commands::Joints& in, trajectory_msgs::msg::JointTrajectory& out);
void toROS(const base::VectorXd& in, std_msgs::msg::Float64MultiArray& out);
void toROS(const base::Pose& in, geometry_msgs::msg::Pose& out);
void toROS(const base::Twist& in, geometry_msgs::msg::Twist& out);
void toROS(const base::Acceleration& in, geometry_msgs::msg::Accel& out);
void toROS(const base::samples::RigidBodyStateSE3& in, wbc_msgs::msg::RigidBodyState& out);
void toROS(const base::samples::Joints& in, sensor_msgs::msg::JointState& out);
void toROS(const wbc::TaskConfig& in, wbc_msgs::msg::TaskConfig& out);
void toROS(const wbc::TaskStatus& in, wbc_msgs::msg::TaskStatus& out);
void toROS(const base::Time& in, builtin_interfaces::msg::Time& out);
void toRaw(const wbc_msgs::msg::RigidBodyState& in, std::vector<double> &out);
void toRaw(const trajectory_msgs::msg::JointTrajectory& in, std::vector<double> &out);
void fromRaw(const std::vector<double>& in, base::samples::RigidBodyStateSE3& out);
void fromRaw(const std::vector<double>& in, base::samples::Joints& out);

#endif
