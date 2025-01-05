#include "conversions.hpp"

using namespace std;
using namespace wbc;
using namespace rclcpp;

void fromROS(const sensor_msgs::msg::JointState& in, wbc::types::JointState& out){
     out.resize(in.name.size());
     for(uint i = 0; i < in.name.size(); i++){
         out.position[i] = in.position[i];
         out.velocity[i] = in.velocity[i];
     }
}

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

void toROS(const wbc::types::Pose& pose, const wbc::types::Twist& twist, const wbc::types::SpatialAcceleration& acc, wbc_msgs::msg::RigidBodyState& out){
    toROS(pose,out.pose);
    toROS(twist,out.twist);
    toROS(acc,out.acceleration);
}

void fromROS(const trajectory_msgs::msg::JointTrajectory& in, wbc::types::JointCommand& out){
    if(in.points.size() != 1)
        throw std::runtime_error("Reference trajectory must contain exactly one point");
    out.resize(in.joint_names.size());
    for(size_t i = 0; i < in.joint_names.size(); i++){
        out.position[i] = in.points[0].positions[i];
        out.velocity[i] = in.points[0].velocities[i];
        out.acceleration[i] = in.points[0].accelerations[i];
    }
}

void fromROS(const std_msgs::msg::Float64MultiArray& in, Eigen::VectorXd& out){
    out.resize(in.data.size());
    for(uint i = 0; i < in.data.size(); i++)
        out[i] = in.data[i];
}

void fromROS(const geometry_msgs::msg::WrenchStamped& in, wbc::types::Wrench& out){
    out.force = Eigen::Vector3d(in.wrench.force.x, in.wrench.force.y, in.wrench.force.z);
    out.torque = Eigen::Vector3d(in.wrench.torque.x, in.wrench.torque.y, in.wrench.torque.z);
}

void fromROS(const wbc_msgs::msg::RigidBodyState& in, wbc::types::RigidBodyState& out){
    fromROS(in.pose,out.pose);
    fromROS(in.twist,out.twist);
    fromROS(in.acceleration,out.acceleration);
}

void toROS(const Eigen::VectorXd& in, std_msgs::msg::Float64MultiArray& out){
    out.data.resize(in.size());
    for(uint i = 0; i < in.size(); i++)
        out.data[i] = in[i];
}

void toROS(const wbc::types::JointCommand& in, const vector<string>& joint_names, trajectory_msgs::msg::JointTrajectory& out){
    //out.header.stamp.sec = in.time.toTimeval().tv_sec;
    //out.header.stamp.nanosec = in.time.toTimeval().tv_usec*1000;
    out.joint_names = joint_names;
    out.points.resize(1);
    uint nj = joint_names.size();
    out.points[0].positions.resize(nj);
    out.points[0].velocities.resize(nj);
    out.points[0].accelerations.resize(nj);
    out.points[0].effort.resize(nj);
    for(uint i = 0; i < nj; i++){
        out.points[0].positions[i] = in.position[i];
        out.points[0].velocities[i] = in.velocity[i];
        out.points[0].accelerations[i] = in.acceleration[i];
        out.points[0].effort[i] = in.effort[i];
    }
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

void toROS(const wbc::types::RigidBodyState& in, wbc_msgs::msg::RigidBodyState& out){
    toROS(in.pose, out.pose);
    toROS(in.twist, out.twist);
    toROS(in.acceleration, out.acceleration);
}

void toROS(const wbc::types::JointState& in, const vector<string>& joint_names, sensor_msgs::msg::JointState& out){
    //toROS(in.time, out.header.stamp);
    out.name = joint_names;
    out.position.resize(joint_names.size());
    out.velocity.resize(joint_names.size());
    out.effort.resize(joint_names.size());
    for(uint i = 0; i < joint_names.size(); i++){
        out.position[i] = in.position[i];
        out.velocity[i] = in.velocity[i];
    }
}

void toRaw(const wbc_msgs::msg::RigidBodyState& in, std::vector<double> &out){
    out = {in.pose.position.x,in.pose.position.y,in.pose.position.z,
           in.pose.orientation.x,in.pose.orientation.y,in.pose.orientation.z,in.pose.orientation.w,
           in.twist.linear.x,in.twist.linear.y,in.twist.linear.z,
           in.twist.angular.x,in.twist.angular.y,in.twist.angular.z,
           in.acceleration.linear.x,in.acceleration.linear.y,in.acceleration.linear.z,
           in.acceleration.angular.x,in.acceleration.angular.y,in.acceleration.angular.z};
}

void toRaw(const trajectory_msgs::msg::JointTrajectory& in, std::vector<double> &out){
    uint idx = 0;
    for(uint i = 0; i < in.joint_names.size(); i++){
        out[idx++] = in.points[0].positions[i];
        out[idx++] = in.points[0].velocities[i];
        out[idx++] = in.points[0].accelerations[i];
    }
}

void fromRaw(const std::vector<double>& in, wbc::types::RigidBodyState& out){
    out.pose.orientation = Eigen::Quaterniond(in[6],in[3],in[4],in[5]);
    for(int i = 0; i < 3; i++){
        out.pose.position[i] = in[i];
        out.twist.linear[i] = in[i+7];
        out.twist.angular[i] = in[i+10];
        out.acceleration.linear[i] = in[i+13];
        out.acceleration.angular[i] = in[i+16];
    }
}

void fromRaw(const std::vector<double>& in, wbc::types::JointState& out){
    uint idx = 0;
    out.resize(in.size()/3);
    for(uint i = 0; i < in.size()/3; i++){
        out.position[i] = in[idx++];
        out.velocity[i] = in[idx++];
        out.acceleration[i] = in[idx++];
    }
}

void fromRaw(const std::vector<double>& in, wbc::types::JointCommand& out){
    uint idx = 0;
    out.resize(in.size()/3);    
    for(uint i = 0; i < in.size()/3; i++){
        out.position[i] = in[idx++];
        out.velocity[i] = in[idx++];
        out.effort[i]   = in[idx++];
    }
}

wbc::TaskType fromString(const std::string& in){
    if(in == "spatial_velocity") return wbc::TaskType::spatial_velocity;
    else if(in == "spatial_acceleration") return wbc::TaskType::spatial_acceleration;
    else if(in == "com_velocity") return wbc::TaskType::com_velocity;
    else if(in == "com_acceleration") return wbc::TaskType::com_acceleration;
    else if(in == "joint_velocity") return wbc::TaskType::joint_velocity;
    else if(in == "joint_acceleration") return wbc::TaskType::joint_acceleration;
    else if(in == "wrench_forward") return wbc::TaskType::wrench_forward;
    else return wbc::TaskType::unset;
}
