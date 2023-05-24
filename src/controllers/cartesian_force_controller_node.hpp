#ifndef CARTESIAN_FORCE_CONTROLLER_NODE_HPP
#define CARTESIAN_FORCE_CONTROLLER_NODE_HPP

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <wbc_msgs/msg/rigid_body_state.hpp>

#include "controller_node.hpp"
#include <wbc/controllers/CartesianForcePIDController.hpp>
#include <base/samples/Wrench.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

/**
@brief Force controller in Cartesian space. See <a href="https://github.com/ARC-OPT/wbc/blob/master/src/controllers/CartesianForcePIDController.hpp">here</a> for details.
*/

namespace wbc_ros{

class CartesianForceControllerNode : public ControllerNode{
protected:
    wbc_msgs::msg::RigidBodyState control_output_msg;

    ctrl_lib::CartesianForcePIDController *controller;
    base::samples::Wrench feedback;
    base::samples::Wrench setpoint;
    base::samples::RigidBodyStateSE3 control_output;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_setpoint;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_feedback;
    rclcpp::Publisher<wbc_msgs::msg::RigidBodyState>::SharedPtr control_output_publisher;
public:
    CartesianForceControllerNode(const rclcpp::NodeOptions &options);
    ~CartesianForceControllerNode();

    void setpointCallback(const geometry_msgs::msg::WrenchStamped& msg);
    void feedbackCallback(const geometry_msgs::msg::WrenchStamped& msg);
    virtual void updateController();
};

}

#endif
