#ifndef JOINT_LIMIT_AVOIDANCE_NODE_HPP
#define JOINT_LIMIT_AVOIDANCE_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <base/samples/Joints.hpp>
#include <wbc/controllers/JointLimitAvoidanceController.hpp>

class JointLimitAvoidanceNode{
    enum controllerState{PRE_OPERATIONAL = 0,
                         NO_FEEDBACK,
                         RUNNING};

     std_msgs::String controllerStateToStringMsg(controllerState s){
         std_msgs::String msg;
         switch(s){
             case PRE_OPERATIONAL: msg.data = "PRE_OPERATIONAL";
             case NO_FEEDBACK: msg.data = "NO_FEEDBACK";
             case RUNNING: msg.data = "RUNNING";
         }
         return msg;
     }

protected:
    ros::NodeHandle* nh;
    ros::Publisher control_output_publisher;
    ros::Publisher state_publisher;
    ros::Subscriber sub_feedback;
    sensor_msgs::JointState control_output_msg;

    double control_rate;
    ctrl_lib::JointLimitAvoidanceController* controller;
    bool has_feedback;
    controllerState state;
    base::samples::Joints control_output;
    base::samples::Joints feedback;

public:
    JointLimitAvoidanceNode(int argc, char** argv);
    ~JointLimitAvoidanceNode();

    void feedbackCallback(const sensor_msgs::JointState& msg);
    void update();
    void run();
};

#endif
