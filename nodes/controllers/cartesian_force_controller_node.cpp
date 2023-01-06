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

    if(!ros::param::has("p_gain")){
        ROS_ERROR("WBC parameter p_gain has not been set");
        abort();
    }
    std::vector<double> p_gain;
    ros::param::get("p_gain", p_gain);

    if(!ros::param::has("i_gain")){
        ROS_ERROR("WBC parameter i_gain has not been set");
        abort();
    }
    std::vector<double> i_gain;
    ros::param::get("i_gain", i_gain);

    if(!ros::param::has("d_gain")){
        ROS_ERROR("WBC parameter d_gain has not been set");
        abort();
    }
    std::vector<double> d_gain;
    ros::param::get("d_gain", d_gain);

    if(!ros::param::has("windup")){
        ROS_ERROR("WBC parameter windup has not been set");
        abort();
    }
    std::vector<double> windup;
    ros::param::get("windup", windup);

    PIDCtrlParams pid_params;
    pid_params.p_gain = Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size());
    pid_params.i_gain = Eigen::Map<Eigen::VectorXd>(i_gain.data(),i_gain.size());
    pid_params.d_gain = Eigen::Map<Eigen::VectorXd>(d_gain.data(),d_gain.size());
    pid_params.windup = Eigen::Map<Eigen::VectorXd>(windup.data(),windup.size());

    controller->setPID(pid_params);

    if(!ros::param::has("max_control_output")){
        ROS_ERROR("WBC parameter max_control_output has not been set");
        abort();
    }
    std::vector<double> max_control_output;
    ros::param::get("max_control_output", max_control_output);
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));

    if(!ros::param::has("dead_zone")){
        ROS_ERROR("WBC parameter dead_zone has not been set");
        abort();
    }
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
