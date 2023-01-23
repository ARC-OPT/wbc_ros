#ifndef JOINT_POSITION_CONTROLLER_NODE_HPP
#define JOINT_POSITION_CONTROLLER_NODE_HPP

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "controller_node.hpp"
#include <wbc/controllers/JointPosPDController.hpp>
#include <base/commands/Joints.hpp>

/**
@brief Position controller in joint space. See <a href="https://github.com/ARC-OPT/wbc/blob/master/src/controllers/JointPosPDController.hpp">here</a> for details.
*/
class JointPositionControllerNode : public ControllerNode{
protected:
    trajectory_msgs::JointTrajectory control_output_msg;

    std::vector<std::string> joint_names;
    ctrl_lib::JointPosPDController* controller;
    base::commands::Joints setpoint;
    base::samples::Joints feedback;
    base::commands::Joints control_output;

public:
    JointPositionControllerNode(int argc, char** argv);
    ~JointPositionControllerNode();

    void setpointCallback(const trajectory_msgs::JointTrajectory& msg);
    void feedbackCallback(const sensor_msgs::JointState& msg);
    virtual void updateController();
};

#endif
