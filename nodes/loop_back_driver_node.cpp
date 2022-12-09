#include "loop_back_driver_node.hpp"

using namespace std;

LoopBackDriverNode::LoopBackDriverNode(int argc, char** argv){
    ros::init(argc, argv, "fake_joints");
    nh = new ros::NodeHandle();

    if(!ros::param::has("initial_joint_state")){
        ROS_ERROR("Parameter 'initial_joint_state' has not been set");
        abort();
    }

    XmlRpc::XmlRpcValue xml_rpc_val;
    ros::param::get("initial_joint_state", xml_rpc_val);
    fromROS(xml_rpc_val, joint_state);

    if(!ros::param::has("noise_std_dev")){
        ROS_ERROR("Parameter 'noise_std_dev' has not been set");
        abort();
    }
    ros::param::get("noise_std_dev", noise_std_dev);

    sub_command = nh->subscribe("command", 1, &LoopBackDriverNode::commandCallback, this);
    pub_joint_state = nh->advertise<sensor_msgs::JointState>("joint_states", 1);
}

LoopBackDriverNode::~LoopBackDriverNode(){
    delete nh;
}

void LoopBackDriverNode::fromROS(const XmlRpc::XmlRpcValue& in, sensor_msgs::JointState& out){
    out.name.clear();
    if(in.hasMember("name")){
        for(uint i = 0; i < in["name"].size(); i++)
            out.name.push_back(static_cast<string>(in["name"][i]));
    }
    out.position.clear();
    if(in.hasMember("position")){
        for(uint i = 0; i < in["position"].size(); i++)
            out.position.push_back(static_cast<double>(in["position"][i]));
    }
    out.velocity.clear();
    if(in.hasMember("velocity")){
        for(uint i = 0; i < in["velocity"].size(); i++)
            out.velocity.push_back(static_cast<double>(in["velocity"][i]));
    }
    out.effort.clear();
    if(in.hasMember("effort")){
        for(uint i = 0; i < in["effort"].size(); i++)
            out.effort.push_back(static_cast<double>(in["effort"][i]));
    }
}

void LoopBackDriverNode::commandCallback(const trajectory_msgs::JointTrajectory& msg){
    for(uint i = 0; i < msg.joint_names.size(); i++){
        string n = msg.joint_names[i];
        auto it = find(joint_state.name.begin(), joint_state.name.end(), n);
        if(it == joint_state.name.end()){
            ROS_ERROR("Joint %s is in command but this joint has not been configured in initial joint state", n.c_str());
            abort();
        }
        int idx = it - joint_state.name.begin();
        joint_state.position[idx] = msg.points[0].positions[i] + whiteNoise(noise_std_dev);
        joint_state.velocity[idx] = msg.points[0].velocities[i] + whiteNoise(noise_std_dev);
        joint_state.effort[idx]   = msg.points[0].effort[i] + whiteNoise(noise_std_dev);
    }
}

double LoopBackDriverNode::whiteNoise(const double std_dev){
    double rand_no = ( rand() / ( (double)RAND_MAX ) );
    while( rand_no == 0 )
        rand_no = ( rand() / ( (double)RAND_MAX ) );

    double tmp = cos( ( 2.0 * (double)M_PI ) * rand() / ( (double)RAND_MAX ) );
    return std_dev * sqrt( -2.0 * log( rand_no ) ) * tmp;
}

void LoopBackDriverNode::update(){
    joint_state.header.stamp = ros::Time::now();
    pub_joint_state.publish(joint_state);
}

int main(int argc, char** argv)
{
    LoopBackDriverNode node(argc, argv);

    double rate;
    if(!ros::param::has("control_rate")){
        ROS_ERROR("Parameter 'control_rate' has not been set");
        abort();
    }
    ros::param::get("control_rate", rate);
    ros::Rate loop_rate(rate);
    ROS_INFO("Fake Joint Driver is running");

    while(ros::ok()){
        node.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
