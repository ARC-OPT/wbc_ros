#ifndef JOINT_POSITION_CONTROLLER_NODE_HPP
#define JOINT_POSITION_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <wbc/controllers/JointPosPDController.hpp>
#include <base/commands/Joints.hpp>

class JointPositionControllerNode{
protected:
    ros::NodeHandle* nh;
    ros::Subscriber sub_setpoint;
    ros::Subscriber sub_feedback;
    ros::Publisher control_output_publisher;
    trajectory_msgs::JointTrajectory control_output_msg;

    double control_rate;
    std::vector<std::string> joint_names;
    ctrl_lib::JointPosPDController* controller;
    bool has_setpoint, has_feedback;
    base::commands::Joints setpoint;
    base::samples::Joints feedback;
    base::commands::Joints control_output;
    std::string node_name;

public:
    JointPositionControllerNode(int argc, char** argv);
    ~JointPositionControllerNode();

    void setpointCallback(const trajectory_msgs::JointTrajectory& msg);
    void feedbackCallback(const sensor_msgs::JointState& msg);
    void update();
    void run();
};

#endif
