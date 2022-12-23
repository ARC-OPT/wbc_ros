#ifndef CARTESIAN_POSITION_CONTROLLER_NODE_HPP
#define CARTESIAN_POSITION_CONTROLLER_NODE_HPP

#include <ros/ros.h>

#include <wbc/controllers/CartesianPosPDController.hpp>
#include <wbc_ros/RigidBodyState.h>

#include <base/samples/RigidBodyStateSE3.hpp>

using namespace ctrl_lib;

class CartesianPositionControllerNode{
protected:
   ros::NodeHandle* nh;
   ros::Subscriber sub_setpoint;
   ros::Subscriber sub_feedback;
   ros::Publisher control_output_publisher;
   ros::Publisher state_publisher;
   wbc_ros::RigidBodyState control_output_msg;

   std::string state;
   double control_rate;
   CartesianPosPDController* controller;
   base::samples::RigidBodyStateSE3 feedback;
   base::samples::RigidBodyStateSE3 setpoint;
   base::samples::RigidBodyStateSE3 control_output;
   bool has_setpoint;
   bool has_feedback;

public:
    CartesianPositionControllerNode(int argc, char** argv);
    ~CartesianPositionControllerNode();

    void setpointCallback(const wbc_ros::RigidBodyState& msg);
    void feedbackCallback(const wbc_ros::RigidBodyState& msg);
    void update();
    void run();
};

#endif
