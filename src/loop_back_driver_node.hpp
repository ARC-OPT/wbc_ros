#ifndef LOOP_BACK_DRIVER_NODE_HPP
#define LOOP_BACK_DRIVER_NODE_HPP

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>

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
