#ifndef LOOP_BACK_DRIVER_NODE_HPP
#define LOOP_BACK_DRIVER_NODE_HPP

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wbc_ros{

/**
@brief A simple loopback driver for testing WBC or other controllers. It will set the current position
to the reference position, plus configurable white noise.

Subscribed Topics:
- `command` (`trajectory_msgs/JointTrajectory`): Input command. Must contain only joints which are configured in initial_joint_state. However, it
  may contain less joints

Published Topics:
- `joint_state` (`sensor_msgs/JointState`): The current joint state of all configured joints

Parameters:
- `initial_joint_state` (dict): Initial state (position, velocity, effort) of all joints. Syntax is same as in sensor_msgs/JointState
*/
class LoopBackDriverNode : public rclcpp::Node{
protected:
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_command;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state;
    rclcpp::TimerBase::SharedPtr timer_update;
    double noise_std_dev;
    sensor_msgs::msg::JointState joint_state;

    double whiteNoise(const double std_dev);

public:
    LoopBackDriverNode(const rclcpp::NodeOptions & options);
    ~LoopBackDriverNode();

    void commandCallback(const trajectory_msgs::msg::JointTrajectory& msg);
    void update();
};

}

#endif
