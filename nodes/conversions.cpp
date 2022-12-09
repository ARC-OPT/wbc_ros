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
