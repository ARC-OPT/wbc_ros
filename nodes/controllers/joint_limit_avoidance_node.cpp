#include "joint_limit_avoidance_node.hpp"
#include "../conversions.hpp"

using namespace ctrl_lib;

JointLimitAvoidanceNode::JointLimitAvoidanceNode(int argc, char** argv) : state(PRE_OPERATIONAL), has_feedback(false){
    std::string node_name = "joint_limit_avoidance";
    ros::init(argc, argv, node_name);
    nh = new ros::NodeHandle();

    ROS_INFO("Initialize Controller: %s", node_name.c_str());

    if(!ros::param::has("control_rate")){
        ROS_ERROR("WBC parameter control_rate has not been set");
        abort();
    }
    ros::param::get("control_rate", control_rate);

    XmlRpc::XmlRpcValue value;
    if(!ros::param::has("joint_limits")){
        ROS_ERROR("WBC parameter joint_limits has not been set");
        abort();
    }
    base::JointLimits joint_limits;
    fromROS(value, joint_limits);

    if(!ros::param::has("influence_distance")){
        ROS_ERROR("WBC parameter influence_distance has not been set");
        abort();
    }
    std::vector<double> influence_distance;
    ros::param::get("influence_distance", influence_distance);

    if(!ros::param::has("p_gain")){
        ROS_ERROR("WBC parameter p_gain has not been set");
        abort();
    }
    std::vector<double> p_gain;
    ros::param::get("p_gain", p_gain);

    if(!ros::param::has("max_control_output")){
        ROS_ERROR("WBC parameter max_control_output has not been set");
        abort();
    }
    std::vector<double> max_control_output;
    ros::param::get("max_control_output", max_control_output);

    controller = new JointLimitAvoidanceController(joint_limits, Eigen::Map<Eigen::VectorXd>(influence_distance.data(), influence_distance.size()));
    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));
    controller->setMaxControlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));

    // controller feedback
    sub_feedback = nh->subscribe("feedback", 1, &JointLimitAvoidanceNode::feedbackCallback, this);
    // State
    state_publisher = nh->advertise<std_msgs::String>("state", 1);
    // Ctrl output
    control_output_publisher = nh->advertise<sensor_msgs::JointState>("control_output", 1);
}

JointLimitAvoidanceNode::~JointLimitAvoidanceNode(){
    delete controller;
    delete nh;
}

void JointLimitAvoidanceNode::feedbackCallback(const sensor_msgs::JointState& msg){
    has_feedback = true;
}

void JointLimitAvoidanceNode::update(){
    switch (state) {
        case PRE_OPERATIONAL:{
            state = NO_FEEDBACK;
            break;
        }
        case NO_FEEDBACK:{
            if(has_feedback)
                state = RUNNING;
            break;
        }
        case RUNNING:{
            control_output = controller->update(feedback);
            toROS(control_output, control_output_msg);
            control_output_publisher.publish(control_output_msg);
            break;
        }
        default: break;
    }
    state_publisher.publish(controllerStateToStringMsg(state));
}

void JointLimitAvoidanceNode::run(){
    ros::Rate loop_rate(control_rate);
    ROS_INFO("Joint Limit Avoidance Controller is running");
    while(ros::ok()){
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    JointLimitAvoidanceNode node(argc, argv);
    node.run();
    return 0;
}
