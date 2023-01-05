#ifndef CARTESIAN_RADIAL_POTENTIAL_FIELDS_NODE_HPP
#define CARTESIAN_RADIAL_POTENTIAL_FIELDS_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <wbc_msgs/RigidBodyState.h>
#include <wbc_msgs/RadialPotentialFieldVector.h>

#include <base/samples/RigidBodyStateSE3.hpp>

#include <wbc/controllers/CartesianPotentialFieldsController.hpp>
#include <wbc/controllers/PotentialField.hpp>

#include "../conversions.hpp"

class CartesianRadialPotentialFieldsNode{
    enum controllerState{PRE_OPERATIONAL = 0,
                         NO_FEEDBACK,
                         NO_POTENTIAL_FIELDS,
                         RUNNING};

     std_msgs::String controllerStateToStringMsg(controllerState s){
         std_msgs::String msg;
         switch(s){
             case PRE_OPERATIONAL: msg.data = "PRE_OPERATIONAL";
             case NO_FEEDBACK: msg.data = "NO_FEEDBACK";
             case NO_POTENTIAL_FIELDS: msg.data = "NO_POTENTIAL_FIELDS";
             case RUNNING: msg.data = "RUNNING";
         }
         return msg;
     }
protected:
    ros::NodeHandle* nh;
    ros::Subscriber sub_pot_fields;
    ros::Subscriber sub_feedback;
    ros::Publisher state_publisher;
    ros::Publisher control_output_publisher;
    wbc_msgs::RigidBodyState control_output_msg;

    double control_rate;
    ctrl_lib::CartesianPotentialFieldsController* controller;
    double influence_distance;
    base::samples::RigidBodyStateSE3 feedback;
    base::samples::RigidBodyStateSE3 control_output;
    bool has_feedback;
    bool has_pot_fields;
    controllerState state;
    std::vector<ctrl_lib::PotentialFieldPtr> fields;

public:
    CartesianRadialPotentialFieldsNode(int argc, char** argv);
    ~CartesianRadialPotentialFieldsNode();

    void feedbackCallback(const wbc_msgs::RigidBodyState& msg);
    void potFieldsCallback(const wbc_msgs::RadialPotentialFieldVector& msg);
    void update();
    void run();
};

#endif
