#ifndef CARTESIAN_FORCE_CONTROLLER_NODE_HPP
#define CARTESIAN_FORCE_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <wbc_msgs/RigidBodyState.h>

#include <wbc/controllers/CartesianForcePIDController.hpp>
#include <base/samples/Wrench.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

class CartesianForceControllerNode{
protected:
    ros::NodeHandle* nh;
    ros::Subscriber sub_feedback;
    ros::Subscriber sub_setpoint;
    ros::Publisher control_output_publisher;
    wbc_msgs::RigidBodyState control_output_msg;

    double control_rate;
    ctrl_lib::CartesianForcePIDController *controller;
    base::samples::Wrench feedback;
    base::samples::Wrench setpoint;
    base::samples::RigidBodyStateSE3 control_output;
    bool has_feedback;
    bool has_setpoint;
    std::string node_name;
public:
    CartesianForceControllerNode(int argc, char** argv);
    ~CartesianForceControllerNode();

    void setpointCallback(const geometry_msgs::WrenchStamped& msg);
    void feedbackCallback(const geometry_msgs::WrenchStamped& msg);
    void update();
    void run();
};

#endif
