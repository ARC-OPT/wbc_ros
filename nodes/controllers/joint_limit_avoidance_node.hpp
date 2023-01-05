#ifndef JOINT_LIMIT_AVOIDANCE_NODE_HPP
#define JOINT_LIMIT_AVOIDANCE_NODE_HPP

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <base/samples/Joints.hpp>
#include <wbc/controllers/JointLimitAvoidanceController.hpp>

class JointLimitAvoidanceNode{
protected:
    ros::NodeHandle* nh;
    ros::Publisher control_output_publisher;
    ros::Subscriber sub_feedback;
    trajectory_msgs::JointTrajectory control_output_msg;

    double control_rate;
    ctrl_lib::JointLimitAvoidanceController* controller;
    bool has_feedback;
    base::samples::Joints control_output;
    base::samples::Joints feedback;
    std::string node_name;

public:
    JointLimitAvoidanceNode(int argc, char** argv);
    ~JointLimitAvoidanceNode();

    void feedbackCallback(const sensor_msgs::JointState& msg);
    void update();
    void run();
};

#endif
