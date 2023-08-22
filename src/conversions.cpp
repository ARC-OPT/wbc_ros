#include "conversions.hpp"

using namespace std;
using namespace wbc;
using namespace rclcpp;

void fromROS(const sensor_msgs::msg::JointState& in, base::samples::Joints& out){
     out.time.fromSeconds(Time(in.header.stamp).seconds());
     out.time = base::Time::now();
     out.resize(in.name.size());
     for(uint i = 0; i < in.name.size(); i++){
         out.names[i]    = in.name[i];
         out[i].position = in.position[i];
         out[i].speed    = in.velocity[i];
         out[i].effort   = in.effort[i];
     }
}

void fromROS(const geometry_msgs::msg::Pose& in, base::Pose& out){
    out.position = base::Vector3d(in.position.x,in.position.y,in.position.z);
    out.orientation = base::Quaterniond(in.orientation.w,in.orientation.x,in.orientation.y,in.orientation.z);
}

void fromROS(const geometry_msgs::msg::Twist& in, base::Twist& out){
    out.linear = base::Vector3d(in.linear.x,in.linear.y,in.linear.z);
    out.angular = base::Vector3d(in.angular.x,in.angular.y,in.angular.z);
}

void fromROS(const geometry_msgs::msg::Accel& in, base::Acceleration& out){
    out.linear = base::Vector3d(in.linear.x,in.linear.y,in.linear.z);
    out.angular = base::Vector3d(in.angular.x,in.angular.y,in.angular.z);
}

void fromROS(const wbc_msgs::msg::RigidBodyState& in, base::samples::RigidBodyStateSE3& out){
    out.time.fromSeconds(Time(in.header.stamp).seconds());
    out.frame_id = in.header.frame_id;
    fromROS(in.pose,out.pose);
    fromROS(in.twist,out.twist);
    fromROS(in.acceleration,out.acceleration);
}

void fromROS(const trajectory_msgs::msg::JointTrajectory& in, base::samples::Joints& out){
    if(in.points.size() != 1)
        throw std::runtime_error("Reference trajectory must contain exactly one point");
    out.time.fromSeconds(Time(in.header.stamp).seconds());
    out.resize(in.joint_names.size());
    out.names = in.joint_names;
    for(size_t i = 0; i < in.joint_names.size(); i++){
        out[i].position = in.points[0].positions[i];
        out[i].speed = in.points[0].velocities[i];
        out[i].acceleration = in.points[0].accelerations[i];
    }
}

void fromROS(const std_msgs::msg::Float64MultiArray& in, base::VectorXd& out){
    out.resize(in.data.size());
    for(uint i = 0; i < in.data.size(); i++)
        out[i] = in.data[i];
}

void fromROS(const std_msgs::msg::Float64MultiArray& in, const vector<string>& joint_names, wbc::JointWeights& out){
    out.names = joint_names;
    out.elements.resize(in.data.size());
    for(uint i = 0; i < in.data.size(); i++)
        out[i] = in.data[i];
}

void fromROS(const geometry_msgs::msg::WrenchStamped& in, base::samples::Wrench& out){
    out.time.fromSeconds(Time(in.header.stamp).seconds());
    out.frame_id = in.header.frame_id;
    out.force = base::Vector3d(in.wrench.force.x, in.wrench.force.y, in.wrench.force.z);
    out.torque = base::Vector3d(in.wrench.torque.x, in.wrench.torque.y, in.wrench.torque.z);
}

void toROS(const base::VectorXd& in, std_msgs::msg::Float64MultiArray& out){
    out.data.resize(in.size());
    for(uint i = 0; i < in.size(); i++)
        out.data[i] = in[i];
}

void toROS(const base::commands::Joints& in, trajectory_msgs::msg::JointTrajectory& out){
    out.header.stamp.sec = in.time.toTimeval().tv_sec;
    out.header.stamp.nanosec = in.time.toTimeval().tv_usec*1000;
    out.joint_names = in.names;
    out.points.resize(1);
    uint nj = in.names.size();
    out.points[0].positions.resize(nj);
    out.points[0].velocities.resize(nj);
    out.points[0].accelerations.resize(nj);
    out.points[0].effort.resize(nj);
    for(uint i = 0; i < nj; i++){
        out.points[0].positions[i] = in[i].position;
        out.points[0].velocities[i] = in[i].speed;
        out.points[0].accelerations[i] = in[i].acceleration;
        out.points[0].effort[i] = in[i].effort;
    }
}

void toROS(const base::Pose& in, geometry_msgs::msg::Pose& out){
    out.position.x = in.position[0];
    out.position.y = in.position[1];
    out.position.z = in.position[2];
    out.orientation.w = in.orientation.w();
    out.orientation.x = in.orientation.x();
    out.orientation.y = in.orientation.y();
    out.orientation.z = in.orientation.z();
}

void toROS(const base::Twist& in, geometry_msgs::msg::Twist& out){
    out.linear.x = in.linear[0];
    out.linear.y = in.linear[1];
    out.linear.z = in.linear[2];
    out.angular.x = in.angular[0];
    out.angular.y = in.angular[1];
    out.angular.z = in.angular[2];
}

void toROS(const base::Acceleration& in, geometry_msgs::msg::Accel& out){
    out.linear.x = in.linear[0];
    out.linear.y = in.linear[1];
    out.linear.z = in.linear[2];
    out.angular.x = in.angular[0];
    out.angular.y = in.angular[1];
    out.angular.z = in.angular[2];
}

void toROS(const base::samples::RigidBodyStateSE3& in, wbc_msgs::msg::RigidBodyState& out){
    toROS(in.time, out.header.stamp);
    out.header.frame_id = in.frame_id;
    toROS(in.pose, out.pose);
    toROS(in.twist, out.twist);
    toROS(in.acceleration, out.acceleration);
}

void toROS(const base::samples::Joints& in, sensor_msgs::msg::JointState& out){
    toROS(in.time, out.header.stamp);
    out.name = in.names;
    out.position.resize(in.size());
    out.velocity.resize(in.size());
    out.effort.resize(in.size());
    for(uint i = 0; i < in.size(); i++){
        out.position[i] = in[i].position;
        out.velocity[i] = in[i].speed;
        out.effort[i] = in[i].effort;
    }
}

void toROS(const TaskConfig& in, wbc_msgs::msg::TaskConfig& out){
    out.name = in.name;
    if(in.type == cart)
        out.type = "cart";
    else
        out.type = "jnt";
    out.priority = (uint)in.priority;
    out.weights.resize(in.weights.size());
    for(uint i = 0; i < in.weights.size(); i++)
        out.weights[i] = in.weights[i];
    out.activation = in.activation;
    out.timeout = in.timeout;
    out.joint_names.resize(in.joint_names.size());
    for(uint i = 0; i < in.joint_names.size(); i++)
        out.joint_names[i] = in.joint_names[i];
    out.root = in.root;
    out.tip = in.tip;
    out.ref_frame = in.ref_frame;
}

void toROS(const TaskStatus& in, wbc_msgs::msg::TaskStatus& out){
    toROS(in.time, out.header.stamp);
    toROS(in.config,out.config);
    out.activation = in.activation;
    out.weights.resize(in.weights.size());
    for(uint i = 0; i < in.weights.size(); i++)
        out.weights[i] = in.weights[i];
    out.timeout = (uint)in.timeout;
    out.y_ref.resize(in.y_ref.size());
    for(uint i = 0; i < in.y_ref.size(); i++)
        out.y_ref[i] = in.y_ref[i];
    out.y_solution.resize(in.y_solution.size());
    for(uint i = 0; i < in.y_solution.size(); i++)
        out.y_solution[i] = in.y_solution[i];
    out.y.resize(in.y.size());
    for(uint i = 0; i < in.y.size(); i++)
        out.y[i] = in.y[i];
}

void toROS(const base::Time& in, builtin_interfaces::msg::Time& out){
    out = Time(in.toSeconds(), in.toMicroseconds()*1000);
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

void fromRaw(const std::vector<double>& in, base::samples::RigidBodyStateSE3& out){
    out.pose.orientation = base::Orientation(in[6],in[3],in[4],in[5]);
    for(int i = 0; i < 3; i++){
        out.pose.position[i] = in[i];
        out.twist.linear[i] = in[i+7];
        out.twist.angular[i] = in[i+10];
        out.acceleration.linear[i] = in[i+13];
        out.acceleration.angular[i] = in[i+16];
    }
}

void fromRaw(const std::vector<double>& in, base::samples::Joints& out){
    uint idx = 0;
    for(uint i = 0; i < out.size(); i++){
        out[i].position = in[idx++];
        out[i].speed = in[idx++];
        out[i].acceleration = in[idx++];
    }
}
