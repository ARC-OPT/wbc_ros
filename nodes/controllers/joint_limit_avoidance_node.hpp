#ifndef JOINT_LIMIT_AVOIDANCE_NODE_HPP
#define JOINT_LIMIT_AVOIDANCE_NODE_HPP

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include "controller_node.hpp"
#include <base/samples/Joints.hpp>
#include <wbc/controllers/JointLimitAvoidanceController.hpp>

class JointLimitAvoidanceNode : public ControllerNode{
protected:
    trajectory_msgs::JointTrajectory control_output_msg;

    ctrl_lib::JointLimitAvoidanceController* controller;
    base::samples::Joints control_output;
    base::samples::Joints feedback;

public:
    JointLimitAvoidanceNode(int argc, char** argv);
    ~JointLimitAvoidanceNode();

    void feedbackCallback(const sensor_msgs::JointState& msg);
    virtual void updateController();
};

#endif
