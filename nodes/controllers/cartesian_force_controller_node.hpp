#ifndef CARTESIAN_FORCE_CONTROLLER_NODE_HPP
#define CARTESIAN_FORCE_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include "../conversions.hpp"

#include <wbc/controllers/CartesianForcePIDController.hpp>
#include <base/samples/Wrench.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

class CartesianForceControllerNode{
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
    ros::Subscriber sub_feedback;
    ros::Subscriber sub_setpoint;
    ros::Publisher control_output_publisher;
    ros::Publisher state_publisher;
    wbc_ros::RigidBodyState control_output_msg;

    double control_rate;
    ctrl_lib::CartesianForcePIDController *controller;
    base::samples::Wrench feedback;
    base::samples::Wrench setpoint;
    base::samples::RigidBodyStateSE3 control_output;
    controllerState state;
    bool has_feedback;
    bool has_setpoint;
public:
    CartesianForceControllerNode(int argc, char** argv);
    ~CartesianForceControllerNode();

    void setpointCallback(const geometry_msgs::WrenchStamped& msg);
    void feedbackCallback(const geometry_msgs::WrenchStamped& msg);
    void update();
    void run();
};

#endif
