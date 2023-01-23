#ifndef LOOP_BACK_DRIVER_NODE_HPP
#define LOOP_BACK_DRIVER_NODE_HPP

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>

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
class LoopBackDriverNode{
protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub_command;
    ros::Publisher pub_joint_state;
    double noise_std_dev;
    sensor_msgs::JointState joint_state;

    void fromXmlRpc(const XmlRpc::XmlRpcValue& in, sensor_msgs::JointState& out);
    double whiteNoise(const double std_dev);

public:
    LoopBackDriverNode(int argc, char** argv);
    ~LoopBackDriverNode();

    void commandCallback(const trajectory_msgs::JointTrajectory& msg);
    void update();
};

#endif
