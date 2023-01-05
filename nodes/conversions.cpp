#include "conversions.hpp"

using namespace std;
using namespace wbc;

void fromROS(const XmlRpc::XmlRpcValue& in, base::RigidBodyStateSE3 &out){
    XmlRpc::XmlRpcValue pos   = in["pose"]["position"];
    XmlRpc::XmlRpcValue ori   = in["pose"]["orientation"];
    XmlRpc::XmlRpcValue twist = in["twist"];
    XmlRpc::XmlRpcValue acc   = in["acceleration"];
    double x = static_cast<double>(ori["im"][0]);
    double y = static_cast<double>(ori["im"][1]);
    double z = static_cast<double>(ori["im"][2]);
    double w = static_cast<double>(ori["re"]);
    for(int i = 0; i < 3; i++){
        out.pose.position[i]        = static_cast<double>(pos["data"][i]);
        out.twist.linear[i]         = static_cast<double>(twist["linear"]["data"][i]);
        out.twist.angular[i]        = static_cast<double>(twist["angular"]["data"][i]);
        out.acceleration.linear[i]  = static_cast<double>(acc["linear"]["data"][i]);
        out.acceleration.angular[i] = static_cast<double>(acc["angular"]["data"][i]);
    }
    out.pose.orientation = base::Quaterniond(w,x,y,z);
}

void fromROS(const XmlRpc::XmlRpcValue& in, ActiveContact& out){
    if(in.hasMember("active"))
        out.active = static_cast<int>(in["active"]);
    if(in.hasMember("mu"))
        out.active = static_cast<double>(in["mu"]);
    if(in.hasMember("wx"))
        out.active = static_cast<double>(in["wx"]);
    if(in.hasMember("wy"))
        out.active = static_cast<double>(in["wy"]);
}

void fromROS(const XmlRpc::XmlRpcValue& in, RobotModelConfig& out){
    if(in.hasMember("file"))
        out.file = static_cast<string>(in["file"]);
    if(in.hasMember("submechanism_file"))
        out.submechanism_file = static_cast<string>(in["submechanism_file"]);
    if(in.hasMember("type"))
        out.type = static_cast<string>(in["type"]);
    if(in.hasMember("floating_base"))
        out.floating_base = static_cast<bool>(in["floating_base"]);
    if(in.hasMember("contact_points")){
        for(int i = 0; i < in["contact_points"]["names"].size(); i++)
            out.contact_points.names.push_back(static_cast<string>(in["contact_points"]["names"][i]));
        ActiveContact c;
        for(int i = 0; i < in["contact_points"]["elements"].size(); i++){
            fromROS(in["contact_points"]["elements"][i], c);
            out.contact_points.elements.push_back(c);
        }
   }
}

void fromROS(const XmlRpc::XmlRpcValue& in, vector<TaskConfig>& out){
    out.clear();
    for(int i = 0; i < in.size(); i++){
        TaskConfig c;
        if(in[i].hasMember("name"))
            c.name = static_cast<string>(in[i]["name"]);
        if(in[i].hasMember("priority"))
            c.priority = static_cast<int>(in[i]["priority"]);
        if(in[i].hasMember("type")){
            string constraint_type = static_cast<string>(in[i]["type"]);
            if(constraint_type == "cart"){
                c.type      = cart;
                if(in[i].hasMember("root"))
                    c.root = static_cast<string>(in[i]["root"]);
                if(in[i].hasMember("tip"))
                    c.tip = static_cast<string>(in[i]["tip"]);
                if(in[i].hasMember("ref_frame"))
                    c.ref_frame = static_cast<string>(in[i]["ref_frame"]);
                if(in[i].hasMember("weights")){
                    for(int j = 0; j < in[i]["weights"].size(); j++)
                        c.weights.push_back(static_cast<double>(in[i]["weights"][j]));
                }
            }
            else if(constraint_type == "jnt"){
                c.type = jnt;
                if(in[i].hasMember("joint_names")){
                    for(int j = 0; j < in[i]["joint_names"].size(); j++)
                        c.joint_names.push_back(static_cast<string>(in[i]["joint_names"][j]));
                }
                if(in[i].hasMember("weights")){
                    for(int j = 0; j < in[i]["weights"].size(); j++)
                        c.weights.push_back(static_cast<double>(in[i]["weights"][j]));
                }
           }
       }
       if(in[i].hasMember("activation"))
           c.activation = static_cast<double>(in[i]["activation"]);
       if(in[i].hasMember("timeout"))
           c.timeout = static_cast<double>(in[i]["timeout"]);

       out.push_back(c);
   }
}

void fromROS(const XmlRpc::XmlRpcValue& in, std::vector<string>& out){
    out.clear();
    for(int i = 0; i < in.size(); i++){
        out.push_back(static_cast<string>(in[i]));
    }
}

void fromROS(const XmlRpc::XmlRpcValue& in, base::JointLimits& out){

}

void fromROS(const sensor_msgs::JointState& in, base::samples::Joints& out){
     out.time = base::Time::now();
     out.resize(in.name.size());
     for(uint i = 0; i < in.name.size(); i++){
         out.names[i]    = in.name[i];
         out[i].position = in.position[i];
         out[i].speed    = in.velocity[i];
         out[i].effort   = in.effort[i];
     }
}

void fromROS(const geometry_msgs::Pose& in, base::Pose& out){
    out.position = base::Vector3d(in.position.x,in.position.y,in.position.z);
    out.orientation = base::Quaterniond(in.orientation.w,in.orientation.x,in.orientation.y,in.orientation.z);
}

void fromROS(const geometry_msgs::Twist& in, base::Twist& out){
    out.linear = base::Vector3d(in.linear.x,in.linear.y,in.linear.z);
    out.angular = base::Vector3d(in.angular.x,in.angular.y,in.angular.z);
}

void fromROS(const geometry_msgs::Accel& in, base::Acceleration& out){
    out.linear = base::Vector3d(in.linear.x,in.linear.y,in.linear.z);
    out.angular = base::Vector3d(in.angular.x,in.angular.y,in.angular.z);
}

void fromROS(const wbc_ros::RigidBodyState& in, base::samples::RigidBodyStateSE3& out){
    out.time.fromSeconds(in.header.stamp.toSec());
    out.frame_id = in.header.frame_id;
    fromROS(in.pose,out.pose);
    fromROS(in.twist,out.twist);
    fromROS(in.acceleration,out.acceleration);
}

void fromROS(const trajectory_msgs::JointTrajectory& in, base::samples::Joints& out){
    if(in.points.size() != 1)
        throw std::runtime_error("Reference trajectory must contain exactly one point");
    out.time.fromSeconds(in.header.stamp.toSec());
    out.resize(in.joint_names.size());
    out.names = in.joint_names;
    for(size_t i = 0; i < in.joint_names.size(); i++){
        out[i].position = in.points[0].positions[i];
        out[i].speed = in.points[0].velocities[i];
        out[i].acceleration = in.points[0].accelerations[i];
    }
}

void fromROS(const std_msgs::Float64MultiArray& in, base::VectorXd& out){
    out.resize(in.data.size());
    for(uint i = 0; i < in.data.size(); i++)
        out[i] = in.data[i];
}

void fromROS(const std_msgs::Float64MultiArray& in, const vector<string>& joint_names, wbc::JointWeights& out){
    out.names = joint_names;
    out.elements.resize(in.data.size());
    for(uint i = 0; i < in.data.size(); i++)
        out[i] = in.data[i];
}

void fromROS(const geometry_msgs::WrenchStamped& in, base::samples::Wrench& out){
    out.time.fromSeconds(in.header.stamp.toSec());
    out.frame_id = in.header.frame_id;
    out.force = base::Vector3d(in.wrench.force.x, in.wrench.force.y, in.wrench.force.z);
    out.torque = base::Vector3d(in.wrench.torque.x, in.wrench.torque.y, in.wrench.torque.z);
}

void toROS(const base::commands::Joints& in, trajectory_msgs::JointTrajectory& out){
    out.header.stamp.fromSec(in.time.toSeconds());
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

void toROS(const base::Pose& in, geometry_msgs::Pose& out){
    out.position.x = in.position[0];
    out.position.y = in.position[1];
    out.position.z = in.position[2];
    out.orientation.w = in.orientation.w();
    out.orientation.x = in.orientation.x();
    out.orientation.y = in.orientation.y();
    out.orientation.z = in.orientation.z();
}

void toROS(const base::Twist& in, geometry_msgs::Twist& out){
    out.linear.x = in.linear[0];
    out.linear.y = in.linear[1];
    out.linear.z = in.linear[2];
    out.angular.x = in.angular[0];
    out.angular.y = in.angular[1];
    out.angular.z = in.angular[2];
}

void toROS(const base::Acceleration& in, geometry_msgs::Accel& out){
    out.linear.x = in.linear[0];
    out.linear.y = in.linear[1];
    out.linear.z = in.linear[2];
    out.angular.x = in.angular[0];
    out.angular.y = in.angular[1];
    out.angular.z = in.angular[2];
}

void toROS(const base::samples::RigidBodyStateSE3& in, wbc_ros::RigidBodyState& out){
    out.header.stamp.fromSec(in.time.toSeconds());
    out.header.frame_id = in.frame_id;
    toROS(in.pose, out.pose);
    toROS(in.twist, out.twist);
    toROS(in.acceleration, out.acceleration);
}

void toROS(const base::samples::Joints& in, sensor_msgs::JointState& out){
    out.name = in.names;
    out.header.stamp.fromSec(in.time.toSeconds());
    out.position.resize(in.size());
    out.velocity.resize(in.size());
    out.effort.resize(in.size());
    for(int i = 0; i < in.size(); i++){
        out.position[i] = in[i].position;
        out.velocity[i] = in[i].speed;
        out.effort[i] = in[i].effort;
    }
}

void toROS(const TaskConfig& in, wbc_ros::TaskConfig& out){
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

void toROS(const TaskStatus& in, wbc_ros::TaskStatus& out){
    out.header.stamp.fromSec(in.time.toSeconds());
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
