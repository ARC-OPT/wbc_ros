#ifndef CARTESIAN_FORCE_CONTROLLER_NODE_HPP
#define CARTESIAN_FORCE_CONTROLLER_NODE_HPP

#include <geometry_msgs/WrenchStamped.h>
#include <wbc_msgs/RigidBodyState.h>

#include "controller_node.hpp"
#include <wbc/controllers/CartesianForcePIDController.hpp>
#include <base/samples/Wrench.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

class CartesianForceControllerNode : public ControllerNode{
protected:
    wbc_msgs::RigidBodyState control_output_msg;

    ctrl_lib::CartesianForcePIDController *controller;
    base::samples::Wrench feedback;
    base::samples::Wrench setpoint;
    base::samples::RigidBodyStateSE3 control_output;
public:
    CartesianForceControllerNode(int argc, char** argv);
    ~CartesianForceControllerNode();

    void setpointCallback(const geometry_msgs::WrenchStamped& msg);
    void feedbackCallback(const geometry_msgs::WrenchStamped& msg);
    virtual void updateController();
};

#endif
