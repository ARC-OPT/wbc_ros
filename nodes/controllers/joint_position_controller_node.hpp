#ifndef JOINT_POSITION_CONTROLLER_NODE_HPP
#define JOINT_POSITION_CONTROLLER_NODE_HPP

#include <ros/ros.h>

#include <wbc/controllers/JointPosPDController.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <base/commands/Joints.hpp>

using namespace ctrl_lib;

class JointPositionControllerNode{
protected:
   ros::NodeHandle* nh;
   ros::Subscriber sub_setpoint;
   ros::Subscriber sub_feedback;
   ros::Publisher state_publisher;
   ros::Publisher control_output_publisher;
   trajectory_msgs::JointTrajectory control_output_msg;

   std::string state;
   double control_rate;
   std::vector<std::string> joint_names;
   JointPosPDController* controller;
   bool has_setpoint, has_feedback;
   base::commands::Joints setpoint;
   base::samples::Joints feedback;
   base::commands::Joints control_output;

public:
    JointPositionControllerNode(int argc, char** argv);
    ~JointPositionControllerNode();

    void update();
    void run();

    void setpointCallback(const trajectory_msgs::JointTrajectory& msg);
    void feedbackCallback(const sensor_msgs::JointState& msg);
};

#endif
