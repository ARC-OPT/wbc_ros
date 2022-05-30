#include "wbc_node.hpp"
#include <wbc/scenes/VelocitySceneQuadraticCost.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>

using namespace wbc;
using namespace std;

WbcNode::WbcNode(int argc, char** argv){

    ros::init(argc, argv, "wbc");
    nh = new ros::NodeHandle();
    
    XmlRpc::XmlRpcValue xml_rpc_val;
    ros::param::get("robot_model_config", xml_rpc_val);
    if(!xml_rpc_val.valid()){
        ROS_ERROR("ROS parameter 'robot_model_config' has not been set"); 
        abort();
    }
    RobotModelConfig robot_model_cfg = xmlrpc2RobotModelConfig(xml_rpc_val);
    
    ROS_INFO("Configuring robot model: %s", robot_model_cfg.type.c_str());
    
    PluginLoader::loadPlugin("libwbc-robot_models-" + robot_model_cfg.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(robot_model_cfg.type));    
    if(!robot_model->configure(robot_model_cfg)){
        ROS_ERROR("Failed to configure robot model");
        abort();
    }        
        
    xml_rpc_val.clear();
    ros::param::get("wbc_config", xml_rpc_val);
    if(!xml_rpc_val.valid()){
        ROS_ERROR("ROS parameter 'wbc_config' has not been set"); 
        abort();
    }    
    vector<ConstraintConfig> wbc_config = xmlrpc2WbcConfig(xml_rpc_val);
    
    ROS_INFO("Configuring solver: %s", "qpoases");
    solver = make_shared<QPOASESSolver>();
    
    ROS_INFO("Configuring scene: %s", "VelocitySceneQuadraticCost");
    scene = make_shared<VelocitySceneQuadraticCost>(robot_model, solver);    
    if(!scene->configure(wbc_config)){
        ROS_ERROR("Failed to configure scene");
        abort();
    }
    
    ROS_INFO("Creating interfaces");
    joint_state.resize(robot_model->noOfJoints());
    joint_state.names = robot_model->jointNames(); 
    joint_state_suscriber = nh->subscribe("joint_states", 1, &WbcNode::jointStateCallback, this);
}

WbcNode::~WbcNode(){
    delete nh;
}

base::RigidBodyStateSE3 WbcNode::xmlrpc2RigidBodyStateSE3(const XmlRpc::XmlRpcValue& in){
    base::RigidBodyStateSE3 out;
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
    return out;
}

RobotModelConfig WbcNode::xmlrpc2RobotModelConfig(const XmlRpc::XmlRpcValue& in){

   RobotModelConfig out;
   
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
       out.floating_base_state = xmlrpc2RigidBodyStateSE3(in["floating_base_state"]);
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
   
   return out;
}

vector<ConstraintConfig> WbcNode::xmlrpc2WbcConfig(const XmlRpc::XmlRpcValue& in){

   vector<ConstraintConfig> out;

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

   return out;
}

void WbcNode::jointStateCallback(const sensor_msgs::JointState& msg){
   
   joint_state.time.fromSeconds(msg.header.stamp.toSec());
   for(uint i = 0; i < msg.name.size(); i++){
       joint_state[msg.name[i]].position = msg.position[i];
       joint_state[msg.name[i]].speed = msg.velocity[i];
       joint_state[msg.name[i]].effort = msg.effort[i];
   }
}

void callback(boost::shared_ptr<geometry_msgs::Twist> ref){
}

void WbcNode::update(){
   bool has_joint_state = true;
   for(uint i = 0; i < joint_state.size(); i++){
       if(!joint_state[i].hasPosition())
           has_joint_state = false;
   }      
   if(has_joint_state)
       robot_model->update(joint_state);
}

int main(int argc, char** argv)
{

    WbcNode node(argc, argv);
    /*ros::Subscriber sub;
    for(auto w : wbc_config)
       sub = nh.subscribe("ref_" + w.name, 1, callback);*/
    
    ROS_INFO("Whole-Body Controller is running");
    ros::spin();
    return 0;
}
