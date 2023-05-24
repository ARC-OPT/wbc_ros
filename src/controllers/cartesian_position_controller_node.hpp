#ifndef CARTESIAN_POSITION_CONTROLLER_NODE_HPP
#define CARTESIAN_POSITION_CONTROLLER_NODE_HPP

#include <wbc_msgs/msg/rigid_body_state.hpp>

#include "controller_node.hpp"
#include <wbc/controllers/CartesianPosPDController.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc_ros{

/**
@brief Position controller in Cartesian space. See <a href="https://github.com/ARC-OPT/wbc/blob/master/src/controllers/CartesianPosPDController.hpp">here</a> for details.
*/
class CartesianPositionControllerNode : public ControllerNode{
protected:
    wbc_msgs::msg::RigidBodyState control_output_msg;

    ctrl_lib::CartesianPosPDController* controller;
    base::samples::RigidBodyStateSE3 feedback;
    base::samples::RigidBodyStateSE3 setpoint;
    base::samples::RigidBodyStateSE3 control_output;

    rclcpp::Subscription<wbc_msgs::msg::RigidBodyState>::SharedPtr sub_setpoint;
    rclcpp::Subscription<wbc_msgs::msg::RigidBodyState>::SharedPtr sub_feedback;
    rclcpp::Publisher<wbc_msgs::msg::RigidBodyState>::SharedPtr control_output_publisher;

public:
    CartesianPositionControllerNode(const rclcpp::NodeOptions &options);
    ~CartesianPositionControllerNode();

    void setpointCallback(const wbc_msgs::msg::RigidBodyState& msg);
    void feedbackCallback(const wbc_msgs::msg::RigidBodyState& msg);
    virtual void updateController();
};

}

#endif
