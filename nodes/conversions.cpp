#include "conversions.hpp"

using namespace std;

namespace wbc{

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

void fromROS(const XmlRpc::XmlRpcValue& in, RobotModelConfig& out){

   // RobotModelConfig::file is the only required entry, all others are optional
   if(!in.hasMember("file")){
       ROS_ERROR("Invalid Robot Model Config: file");
       abort();
   }
   out.file = static_cast<string>(in["file"]);

   if(in.hasMember("submechanism_file"))
       out.submechanism_file = static_cast<string>(in["submechanism_file"]);
   if(in.hasMember("type"))
       out.type = static_cast<string>(in["type"]);
   if(in.hasMember("joint_names")){
       for(int i = 0; i < in["joint_names"].size(); i++)
           out.joint_names.push_back(static_cast<string>(in["joint_names"][i]));
   }
   if(in.hasMember("actuated_joint_names")){
       for(int i = 0; i < in["actuated_joint_names"].size(); i++)
           out.actuated_joint_names.push_back(static_cast<string>(in["actuated_joint_names"][i]));
   }
   if(in.hasMember("floating_base"))
       out.floating_base = static_cast<bool>(in["floating_base"]);
   if(in.hasMember("world_frame_id"))
       out.world_frame_id = static_cast<string>(in["world_frame_id"]);
   if(in.hasMember("floating_base_state"))
       fromROS(in["floating_base_state"], out.floating_base_state);
   if(in.hasMember("contact_points")){
       for(int i = 0; i < in["contact_points"]["names"].size(); i++)
           out.contact_points.names.push_back(static_cast<string>(in["contact_points"]["names"][i]));
       for(int i = 0; i < in["contact_points"]["elements"].size(); i++)
           out.contact_points.elements.push_back(static_cast<int>(in["contact_points"]["elements"][i]));
   }
   if(in.hasMember("joint_blacklist")){
       for(int i = 0; i < in["joint_blacklist"].size(); i++)
           out.joint_blacklist.push_back(static_cast<string>(in["joint_blacklist"][i]));
   }
}

void fromROS(const XmlRpc::XmlRpcValue& in, vector<ConstraintConfig>& out){

   out.clear();
   for(int i = 0; i < in.size(); i++){
       ConstraintConfig c;
       c.name     = static_cast<string>(in[i]["name"]);
       c.priority = static_cast<int>(in[i]["priority"]);
       string constraint_type = static_cast<string>(in[i]["type"]);
       if(constraint_type == "cart"){
           c.type      = cart;
           c.root      = static_cast<string>(in[i]["root"]);
           c.tip       = static_cast<string>(in[i]["tip"]);
           c.ref_frame = static_cast<string>(in[i]["ref_frame"]);
           for(int j = 0; j < in[i]["weights"].size(); j++)
               c.weights.push_back(static_cast<double>(in[i]["weights"][j]));
       }
       else if(constraint_type == "jnt"){
           c.type = jnt;
           for(int j = 0; j < in[i]["joint_names"].size(); j++)
               c.joint_names.push_back(static_cast<string>(in[i]["joint_names"][j]));
           for(int j = 0; j < in[i]["weights"].size(); j++)
               c.weights.push_back(static_cast<double>(in[i]["weights"][j]));
       }
       if(in[i].hasMember("activation"))
           c.activation = static_cast<double>(in[i]["activation"]);
       if(in[i].hasMember("timeout"))
           c.timeout = static_cast<double>(in[i]["timeout"]);

       out.push_back(c);
   }
}

void fromROS(const sensor_msgs::JointState& in, base::samples::Joints& out){
     out.time.fromSeconds(in.header.stamp.toSec());
     for(uint i = 0; i < in.name.size(); i++){
         out[in.name[i]].position = in.position[i];
         out[in.name[i]].speed = in.velocity[i];
         out[in.name[i]].effort = in.effort[i];
     }
}

void fromROS(const geometry_msgs::TwistStamped& in, base::samples::RigidBodyStateSE3& out){
    out.time.fromSeconds(in.header.stamp.toSec());
    out.frame_id = in.header.frame_id;
    out.twist.linear = base::Vector3d(in.twist.linear.x,in.twist.linear.y,in.twist.linear.z);
    out.twist.angular = base::Vector3d(in.twist.angular.x,in.twist.angular.y,in.twist.angular.z);
    out.acceleration.linear = base::Vector3d(in.twist.linear.x,in.twist.linear.y,in.twist.linear.z);
    out.acceleration.angular = base::Vector3d(in.twist.angular.x,in.twist.angular.y,in.twist.angular.z);
}

void fromROS(const trajectory_msgs::JointTrajectory& in, base::commands::Joints& out){
    if(in.points.size() != 1)
        throw std::runtime_error("Reference trajectory must contain exactly one point");
    out.time.fromSeconds(in.header.stamp.toSec());
    out.names = in.joint_names;
    for(size_t i = 0; i < in.joint_names.size(); i++){
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

}
