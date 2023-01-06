#ifndef CARTESIAN_POSITION_CONTROLLER_NODE_HPP
#define CARTESIAN_POSITION_CONTROLLER_NODE_HPP

#include <wbc_msgs/RigidBodyState.h>

#include "controller_node.hpp"
#include <wbc/controllers/CartesianPosPDController.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

class CartesianPositionControllerNode : public ControllerNode{
protected:
    wbc_msgs::RigidBodyState control_output_msg;

    ctrl_lib::CartesianPosPDController* controller;
    base::samples::RigidBodyStateSE3 feedback;
    base::samples::RigidBodyStateSE3 setpoint;
    base::samples::RigidBodyStateSE3 control_output;

public:
    CartesianPositionControllerNode(int argc, char** argv);
    ~CartesianPositionControllerNode();

    void setpointCallback(const wbc_msgs::RigidBodyState& msg);
    void feedbackCallback(const wbc_msgs::RigidBodyState& msg);
    virtual void updateController();
};

#endif
