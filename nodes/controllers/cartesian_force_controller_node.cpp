#include "cartesian_force_controller_node.hpp"

using namespace ctrl_lib;

CartesianForceControllerNode::CartesianForceControllerNode(int argc, char** argv) : state(PRE_OPERATIONAL), has_feedback(false),has_setpoint(false){
    std::string node_name = "cartesian_force_controller";
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
    // State
    state_publisher = nh->advertise<std_msgs::String>("state", 1);
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
    switch (state) {
        case PRE_OPERATIONAL:{
            state = NO_FEEDBACK;
            break;
        }
        case NO_FEEDBACK:{
            if(has_feedback)
                state = NO_SETPOINT;
            break;
        }
        case NO_SETPOINT:{
            if(has_setpoint)
                state = RUNNING;
            break;
        }
        case RUNNING:{
            control_output = controller->update(setpoint, feedback, 1/control_rate);
            toROS(control_output, control_output_msg);
            control_output_publisher.publish(control_output_msg);
            break;
        }
        default: break;
    }
    state_publisher.publish(controllerStateToStringMsg(state));
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

int main(int argc, char** argv)
{
    CartesianForceControllerNode node(argc, argv);
    node.run();
    return 0;
}
