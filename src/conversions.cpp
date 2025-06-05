#include "conversions.hpp"

using namespace std;
using namespace wbc;
using namespace rclcpp;

void fromROS(const geometry_msgs::msg::Pose& in, wbc::types::Pose& out){
    out.position = Eigen::Vector3d(in.position.x,in.position.y,in.position.z);
    out.orientation = Eigen::Quaterniond(in.orientation.w,in.orientation.x,in.orientation.y,in.orientation.z);
}

void fromROS(const geometry_msgs::msg::Twist& in, wbc::types::Twist& out){
    out.linear = Eigen::Vector3d(in.linear.x,in.linear.y,in.linear.z);
    out.angular = Eigen::Vector3d(in.angular.x,in.angular.y,in.angular.z);
}

void fromROS(const geometry_msgs::msg::Accel& in, wbc::types::SpatialAcceleration& out){
    out.linear = Eigen::Vector3d(in.linear.x,in.linear.y,in.linear.z);
    out.angular = Eigen::Vector3d(in.angular.x,in.angular.y,in.angular.z);
}

void fromROS(const geometry_msgs::msg::Wrench& in, wbc::types::Wrench& out){
    out.force = Eigen::Vector3d(in.force.x, in.force.y, in.force.z);
    out.torque = Eigen::Vector3d(in.torque.x, in.torque.y, in.torque.z);
}

void fromROS(const robot_control_msgs::msg::JointState& in, const std::vector<int>& joint_indices, wbc::types::JointState& out){
    assert(joint_indices.size() == in.position.size());
    out.position.resize(in.position.size());
    out.velocity.resize(in.velocity.size());
    out.acceleration.resize(in.velocity.size());
    for(uint i = 0; i < in.position.size(); i++)
        out.position[joint_indices[i]]     = in.position[i];
    for(uint i = 0; i < in.velocity.size(); i++)
        out.velocity[joint_indices[i]]     = in.velocity[i];
    for(uint i = 0; i < in.acceleration.size(); i++)
        out.acceleration[joint_indices[i]] = in.acceleration[i];
}

void fromROS(const robot_control_msgs::msg::RigidBodyState& in, wbc::types::RigidBodyState& out){
    fromROS(in.pose,out.pose);
    fromROS(in.twist,out.twist);
    fromROS(in.acceleration,out.acceleration);
}

void fromROS(const robot_control_msgs::msg::JointCommand& in, const std::vector<int>& joint_indices, wbc::types::JointCommand& out){
    out.position.resize(in.position.size());
    out.velocity.resize(in.velocity.size());
    out.acceleration.resize(in.acceleration.size());
    out.effort.resize(in.effort.size());
    for(uint i = 0; i < in.position.size(); i++)
        out.position[joint_indices[i]] = in.position[i];
    for(uint i = 0; i < in.velocity.size(); i++)
        out.velocity[joint_indices[i]] = in.velocity[i];
    for(uint i = 0; i < in.acceleration.size(); i++)
        out.acceleration[joint_indices[i]] = in.acceleration[i];
    for(uint i = 0; i < in.effort.size(); i++)
        out.effort[joint_indices[i]] = in.effort[i];
}

void fromROS(const robot_control_msgs::msg::Contacts& in, std::vector<wbc::types::Contact>& out){
    out.resize(in.active.size());
    for(uint i = 0; i < in.active.size(); i++)
        out[i].active = (int)in.active[i];
}

void fromROS(const std_msgs::msg::Float64MultiArray& in, Eigen::VectorXd& out){
    out.resize(in.data.size());
    for(uint i = 0; i < in.data.size(); i++)
        out[i] = in.data[i];
}

void fromROS(const robot_control_msgs::msg::RobotState& in, const std::vector<int>& joint_indices, wbc::types::JointState &joint_state_out, wbc::types::RigidBodyState& floating_base_state_out){
    fromROS(in.joint_state, joint_indices, joint_state_out);
    fromROS(in.floating_base_state, floating_base_state_out);
}

void toROS(const wbc::types::RigidBodyState& in, robot_control_msgs::msg::RigidBodyState& out){
    toROS(in.pose, out.pose);
    toROS(in.twist, out.twist);
    toROS(in.acceleration, out.acceleration);
}

void toROS(const wbc::types::JointCommand& in, const std::vector<int>& joint_indices, robot_control_msgs::msg::JointCommand& out){
    out.position.resize(in.position.size());
    out.velocity.resize(in.velocity.size());
    out.acceleration.resize(in.effort.size());
    out.effort.resize(in.effort.size());
    for(uint i = 0; i < in.position.size(); i++)
        out.position[i] = in.position[joint_indices[i]];
    for(uint i = 0; i < in.position.size(); i++)
        out.velocity[i] = in.velocity[joint_indices[i]];
    for(uint i = 0; i < in.acceleration.size(); i++)
        out.acceleration[i] = in.acceleration[joint_indices[i]];
    for(uint i = 0; i < in.effort.size(); i++)
        out.effort[i] = in.effort[joint_indices[i]];
}

void toROS(const wbc::types::JointCommand& in, const std::vector<double> &kp, const std::vector<double> &kd, const std::vector<int>& joint_indices, robot_control_msgs::msg::JointCommand& out){
    toROS(in, joint_indices, out);
    out.kp.resize(kp.size());
    out.kd.resize(kd.size());    
    for(uint i = 0; i < kp.size(); i++)
        out.kp[i] = kp[joint_indices[i]]; 
    for(uint i = 0; i < kd.size(); i++)
        out.kd[i] = kd[joint_indices[i]];
}

void toROS(const Eigen::VectorXd& in, std_msgs::msg::Float64MultiArray& out){
    out.data.resize(in.size());
    for(uint i = 0; i < in.size(); i++)
        out.data[i] = in[i];
}

void toROS(const wbc::types::Pose& in, geometry_msgs::msg::Pose& out){
    out.position.x = in.position[0];
    out.position.y = in.position[1];
    out.position.z = in.position[2];
    out.orientation.w = in.orientation.w();
    out.orientation.x = in.orientation.x();
    out.orientation.y = in.orientation.y();
    out.orientation.z = in.orientation.z();
}

void toROS(const wbc::types::Twist& in, geometry_msgs::msg::Twist& out){
    out.linear.x = in.linear[0];
    out.linear.y = in.linear[1];
    out.linear.z = in.linear[2];
    out.angular.x = in.angular[0];
    out.angular.y = in.angular[1];
    out.angular.z = in.angular[2];
}

void toROS(const wbc::types::SpatialAcceleration& in, geometry_msgs::msg::Accel& out){
    out.linear.x = in.linear[0];
    out.linear.y = in.linear[1];
    out.linear.z = in.linear[2];
    out.angular.x = in.angular[0];
    out.angular.y = in.angular[1];
    out.angular.z = in.angular[2];
}

void toROS(const wbc::types::Pose& pose, const wbc::types::Twist& twist, const wbc::types::SpatialAcceleration& acc, robot_control_msgs::msg::RigidBodyState& out){
    toROS(pose,out.pose);
    toROS(twist,out.twist);
    toROS(acc,out.acceleration);
}

void toROS(const wbc::types::JointState& in, const std::vector<int>& joint_indices, robot_control_msgs::msg::JointState& out){
    out.position.resize(in.position.size());
    out.velocity.resize(in.velocity.size());
    out.acceleration.resize(in.acceleration.size());
    for(uint i = 0; i < in.position.size(); i++)
        out.position[i] = in.position[joint_indices[i]];
    for(uint i = 0; i < in.velocity.size(); i++)
        out.velocity[i] = in.velocity[joint_indices[i]];
    for(uint i = 0; i < in.acceleration.size(); i++)
        out.acceleration[i] = in.acceleration[joint_indices[i]];
}
