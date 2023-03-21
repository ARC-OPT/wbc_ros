#ifndef JOINT_LIMIT_AVOIDANCE_NODE_HPP
#define JOINT_LIMIT_AVOIDANCE_NODE_HPP

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "controller_node.hpp"
#include <base/samples/Joints.hpp>
#include <wbc/controllers/JointLimitAvoidanceController.hpp>

/**
@brief Joint limit avoidance controller. See <a href="https://github.com/ARC-OPT/wbc/blob/master/src/controllers/JointLimitAvoidanceController.hpp">here</a> for details.
*/
class JointLimitAvoidanceNode : public ControllerNode{
protected:
    trajectory_msgs::msg::JointTrajectory control_output_msg;

    ctrl_lib::JointLimitAvoidanceController* controller;
    base::samples::Joints control_output;
    base::samples::Joints feedback;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr control_output_publisher;

public:
    JointLimitAvoidanceNode(const std::string& node_name);
    ~JointLimitAvoidanceNode();

    void feedbackCallback(const sensor_msgs::msg::JointState& msg);
    virtual void updateController();
};

#endif
