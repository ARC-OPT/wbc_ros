#include "cartesian_position_controller_node.hpp"
#include "../conversions.hpp"
#include <std_msgs/String.h>

using namespace ctrl_lib;

CartesianPositionControllerNode::CartesianPositionControllerNode(int argc, char** argv) : state("NO_FEEDBACK"), has_setpoint(false), has_feedback(false){
    ros::init(argc, argv, "cartesian_position_controller");
    nh = new ros::NodeHandle();

    ROS_INFO("Initialize Controller: %s", "Cartesian Position Controller");

    if(!ros::param::has("control_rate")){
        ROS_ERROR("WBC parameter control_rate has not been set");
        abort();
    }
    ros::param::get("control_rate", control_rate);

    controller = new CartesianPosPDController();

    std::vector<double> p_gain;
    ros::param::get("p_gain", p_gain);
    std::vector<double> d_gain;
    ros::param::get("d_gain", d_gain);
    std::vector<double> ff_gain;
    ros::param::get("ff_gain", ff_gain);
    std::vector<double> max_control_output;
    ros::param::get("max_control_output", max_control_output);
    std::vector<double> dead_zone;
    ros::param::get("dead_zone", dead_zone);

    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));
    controller->setDGain(Eigen::Map<Eigen::VectorXd>(d_gain.data(),d_gain.size()));
    controller->setFFGain(Eigen::Map<Eigen::VectorXd>(ff_gain.data(),ff_gain.size()));
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));
    controller->setDeadZone(Eigen::Map<Eigen::VectorXd>(dead_zone.data(),dead_zone.size()));

    // controller setpoint
    sub_setpoint = nh->subscribe("setpoint", 1, &CartesianPositionControllerNode::setpointCallback, this);
    // controller feedback
    sub_feedback = nh->subscribe("feedback", 1, &CartesianPositionControllerNode::feedbackCallback, this);
    // State
    state_publisher = nh->advertise<std_msgs::String>("state", 1);
    // Ctrl output
    control_output_publisher = nh->advertise<wbc_ros::RigidBodyState>("control_output", 1);
}

CartesianPositionControllerNode::~CartesianPositionControllerNode(){
    delete controller;
}

void CartesianPositionControllerNode::setpointCallback(const wbc_ros::RigidBodyState& msg){
    fromROS(msg, setpoint);
    has_setpoint = true;
}

void CartesianPositionControllerNode::feedbackCallback(const wbc_ros::RigidBodyState& msg){
    fromROS(msg, feedback);
    has_feedback = true;
}

void CartesianPositionControllerNode::update(){
    std_msgs::String msg;
    msg.data = state;
    state_publisher.publish(std_msgs::String(msg));

    if(!has_feedback){
        state = "NO_FEEDBACK";
        return;
    }
    if(!has_setpoint){
        state = "NO_SETPOINT";
        return;
    }
    state = "RUNNING";

    control_output = controller->update(setpoint, feedback);

    toROS(control_output, control_output_msg);
    control_output_publisher.publish(control_output_msg);
}

void CartesianPositionControllerNode::run(){
    ros::Rate loop_rate(control_rate);
    ROS_INFO("Cartesian Position Controller is running");
    while(ros::ok()){
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    CartesianPositionControllerNode node(argc, argv);
    node.run();
    return 0;
}
