#ifndef CARTESIAN_RADIAL_POTENTIAL_FIELDS_NODE_HPP
#define CARTESIAN_RADIAL_POTENTIAL_FIELDS_NODE_HPP

#include <wbc_msgs/RigidBodyState.h>
#include <wbc_msgs/RadialPotentialFieldVector.h>

#include "controller_node.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>
#include <wbc/controllers/CartesianPotentialFieldsController.hpp>
#include <wbc/controllers/PotentialField.hpp>

class CartesianRadialPotentialFieldsNode : public ControllerNode{
protected:
    wbc_msgs::RigidBodyState control_output_msg;

    ctrl_lib::CartesianPotentialFieldsController* controller;
    double influence_distance;
    base::samples::RigidBodyStateSE3 feedback;
    base::samples::RigidBodyStateSE3 control_output;
    std::vector<ctrl_lib::PotentialFieldPtr> fields;

public:
    CartesianRadialPotentialFieldsNode(int argc, char** argv);
    ~CartesianRadialPotentialFieldsNode();

    void feedbackCallback(const wbc_msgs::RigidBodyState& msg);
    void potFieldsCallback(const wbc_msgs::RadialPotentialFieldVector& msg);
    virtual void updateController();
};

#endif
