#include "cartesian_force_controller_node.hpp"
#include "../conversions.hpp"

using namespace ctrl_lib;

CartesianForceControllerNode::CartesianForceControllerNode(int argc, char** argv) : has_feedback(false),has_setpoint(false){
    node_name = "cartesian_force_controller";
    ros::init(argc, argv, node_name);
    nh = new ros::NodeHandle();

    ROS_INFO("Initialize Controller: %s", node_name.c_str());

    if(!ros::param::has("control_rate")){
        ROS_ERROR("WBC parameter control_rate has not been set");
        abort();
    }
    ros::param::get("control_rate", control_rate);

    controller = new CartesianForcePIDController();

    // controller setpoint
    sub_setpoint = nh->subscribe("setpoint", 1, &CartesianForceControllerNode::setpointCallback, this);
    // controller feedback
    sub_feedback = nh->subscribe("feedback", 1, &CartesianForceControllerNode::feedbackCallback, this);
    // Ctrl output
    control_output_publisher = nh->advertise<wbc_msgs::RigidBodyState>("control_output", 1);
}

CartesianForceControllerNode::~CartesianForceControllerNode(){
    delete controller;
    delete nh;
}

void CartesianForceControllerNode::setpointCallback(const geometry_msgs::WrenchStamped& msg){
    fromROS(msg, setpoint);
    has_setpoint = true;
}

void CartesianForceControllerNode::feedbackCallback(const geometry_msgs::WrenchStamped& msg){
    fromROS(msg, feedback);
    has_feedback = true;
}

void CartesianForceControllerNode::update(){
    if(!has_feedback){
        ROS_WARN_DELAYED_THROTTLE(5, "%s: No feedback", node_name.c_str());
        return;
    }
    if(!has_setpoint){
        ROS_DEBUG_DELAYED_THROTTLE(5, "%s: No setpoint", node_name.c_str());
        return;
    }
    control_output = controller->update(setpoint, feedback, 1/control_rate);
    toROS(control_output, control_output_msg);
    control_output_publisher.publish(control_output_msg);
}

void CartesianForceControllerNode::run(){
    ros::Rate loop_rate(control_rate);
    ROS_INFO("Cartesian Force Controller is running");
    while(ros::ok()){
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv){
    CartesianForceControllerNode node(argc, argv);
    node.run();
    return 0;
}
