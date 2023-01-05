#ifndef CARTESIAN_POSITION_CONTROLLER_NODE_HPP
#define CARTESIAN_POSITION_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <wbc_msgs/RigidBodyState.h>

#include <wbc/controllers/CartesianPosPDController.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

class CartesianPositionControllerNode{
protected:
    ros::NodeHandle* nh;
    ros::Subscriber sub_setpoint;
    ros::Subscriber sub_feedback;
    ros::Publisher control_output_publisher;
    wbc_msgs::RigidBodyState control_output_msg;

    std::string node_name;
    double control_rate;
    ctrl_lib::CartesianPosPDController* controller;
    base::samples::RigidBodyStateSE3 feedback;
    base::samples::RigidBodyStateSE3 setpoint;
    base::samples::RigidBodyStateSE3 control_output;
    bool has_setpoint;
    bool has_feedback;

public:
    CartesianPositionControllerNode(int argc, char** argv);
    ~CartesianPositionControllerNode();

    void setpointCallback(const wbc_msgs::RigidBodyState& msg);
    void feedbackCallback(const wbc_msgs::RigidBodyState& msg);
    void update();
    void run();
};

#endif
