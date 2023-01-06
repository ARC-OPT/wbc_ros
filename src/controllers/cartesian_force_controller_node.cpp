#include "cartesian_force_controller_node.hpp"
#include "../conversions.hpp"

using namespace ctrl_lib;

CartesianForceControllerNode::CartesianForceControllerNode(int argc, char** argv) : ControllerNode(argc, argv){

    controller = new CartesianForcePIDController();

    checkParam("p_gain");
    std::vector<double> p_gain;
    ros::param::get("p_gain", p_gain);

    checkParam("i_gain");
    std::vector<double> i_gain;
    ros::param::get("i_gain", i_gain);

    checkParam("d_gain");
    std::vector<double> d_gain;
    ros::param::get("d_gain", d_gain);

    checkParam("windup");
    std::vector<double> windup;
    ros::param::get("windup", windup);

    PIDCtrlParams pid_params;
    pid_params.p_gain = Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size());
    pid_params.i_gain = Eigen::Map<Eigen::VectorXd>(i_gain.data(),i_gain.size());
    pid_params.d_gain = Eigen::Map<Eigen::VectorXd>(d_gain.data(),d_gain.size());
    pid_params.windup = Eigen::Map<Eigen::VectorXd>(windup.data(),windup.size());

    controller->setPID(pid_params);

    checkParam("max_control_output");
    std::vector<double> max_control_output;
    ros::param::get("max_control_output", max_control_output);
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));

    checkParam("dead_zone");
    std::vector<double> dead_zone;
    ros::param::get("dead_zone", dead_zone);
    controller->setDeadZone(Eigen::Map<Eigen::VectorXd>(dead_zone.data(),dead_zone.size()));

    // controller setpoint
    sub_setpoint = nh->subscribe("setpoint", 1, &CartesianForceControllerNode::setpointCallback, this);
    // controller feedback
    sub_feedback = nh->subscribe("feedback", 1, &CartesianForceControllerNode::feedbackCallback, this);
    // Ctrl output
    control_output_publisher = nh->advertise<wbc_msgs::RigidBodyState>("control_output", 1);
}

CartesianForceControllerNode::~CartesianForceControllerNode(){
    delete controller;
}

void CartesianForceControllerNode::setpointCallback(const geometry_msgs::WrenchStamped& msg){
    fromROS(msg, setpoint);
    has_setpoint = true;
}

void CartesianForceControllerNode::feedbackCallback(const geometry_msgs::WrenchStamped& msg){
    fromROS(msg, feedback);
    has_feedback = true;
}

void CartesianForceControllerNode::updateController(){
    control_output = controller->update(setpoint, feedback, 1/control_rate);
    toROS(control_output, control_output_msg);
    control_output_publisher.publish(control_output_msg);
}

int main(int argc, char** argv){
    CartesianForceControllerNode node(argc, argv);
    node.run();
    return 0;
}
