#ifndef JOINT_POSITION_CONTROLLER_NODE_HPP
#define JOINT_POSITION_CONTROLLER_NODE_HPP

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "controller_node.hpp"
#include <wbc/controllers/JointPosPDController.hpp>
#include <base/commands/Joints.hpp>

/**
@brief Position controller in joint space. See <a href="https://github.com/ARC-OPT/wbc/blob/master/src/controllers/JointPosPDController.hpp">here</a> for details.
*/
class JointPositionControllerNode : public ControllerNode{
protected:
    trajectory_msgs::msg::JointTrajectory control_output_msg;

    std::vector<std::string> joint_names;
    ctrl_lib::JointPosPDController* controller;
    base::commands::Joints setpoint;
    base::samples::Joints feedback;
    base::commands::Joints control_output;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_setpoint;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_feedback;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr control_output_publisher;

public:
    JointPositionControllerNode(const std::string& node_name);
    ~JointPositionControllerNode();

    void setpointCallback(const trajectory_msgs::msg::JointTrajectory& msg);
    void feedbackCallback(const sensor_msgs::msg::JointState& msg);
    virtual void updateController();
};

#endif
