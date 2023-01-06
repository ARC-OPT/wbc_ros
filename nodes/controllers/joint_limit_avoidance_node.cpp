#include "joint_limit_avoidance_node.hpp"
#include "../conversions.hpp"

using namespace ctrl_lib;

JointLimitAvoidanceNode::JointLimitAvoidanceNode(int argc, char** argv) : ControllerNode(argc, argv){

    checkParam("joint_limits");
    XmlRpc::XmlRpcValue value;
    ros::param::get("joint_limits", value);
    base::JointLimits joint_limits;
    fromROS(value, joint_limits);

    checkParam("influence_distance");
    std::vector<double> influence_distance;
    ros::param::get("influence_distance", influence_distance);

    checkParam("p_gain");
    std::vector<double> p_gain;
    ros::param::get("p_gain", p_gain);

    checkParam("max_control_output");
    std::vector<double> max_control_output;
    ros::param::get("max_control_output", max_control_output);

    controller = new JointLimitAvoidanceController(joint_limits, Eigen::Map<Eigen::VectorXd>(influence_distance.data(), influence_distance.size()));
    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));
    controller->setMaxControlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));

    // controller feedback
    sub_feedback = nh->subscribe("feedback", 1, &JointLimitAvoidanceNode::feedbackCallback, this);
    // Ctrl output
    control_output_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("control_output", 1);

    has_setpoint = true; // Controller does not need a setpoint
}

JointLimitAvoidanceNode::~JointLimitAvoidanceNode(){
    delete controller;
}

void JointLimitAvoidanceNode::feedbackCallback(const sensor_msgs::JointState& msg){
    has_feedback = true;
    fromROS(msg, feedback);
}

void JointLimitAvoidanceNode::updateController(){
    control_output = controller->update(feedback);
    toROS(control_output, control_output_msg);
    control_output_publisher.publish(control_output_msg);
}

int main(int argc, char** argv){
    JointLimitAvoidanceNode node(argc, argv);
    node.run();
    return 0;
}
