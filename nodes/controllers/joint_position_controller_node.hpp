#ifndef JOINT_POSITION_CONTROLLER_NODE_HPP
#define JOINT_POSITION_CONTROLLER_NODE_HPP

#include <ros/ros.h>

#include <wbc/controllers/JointPosPDController.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <base/commands/Joints.hpp>
#include <std_msgs/String.h>

using namespace ctrl_lib;

class JointPositionControllerNode{
   enum controllerState{PRE_OPERATIONAL = 0,
                        NO_FEEDBACK,
                        NO_SETPOINT,
                        RUNNING};

    std_msgs::String controllerStateToStringMsg(controllerState s){
        std_msgs::String msg;
        switch(s){
            case PRE_OPERATIONAL: msg.data = "PRE_OPERATIONAL";
            case NO_FEEDBACK: msg.data = "NO_FEEDBACK";
            case NO_SETPOINT: msg.data = "NO_SETPOINT";
            case RUNNING: msg.data = "RUNNING";
         }
        return msg;
    }
protected:
   ros::NodeHandle* nh;
   ros::Subscriber sub_setpoint;
   ros::Subscriber sub_feedback;
   ros::Publisher state_publisher;
   ros::Publisher control_output_publisher;
   trajectory_msgs::JointTrajectory control_output_msg;

   controllerState state;
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
