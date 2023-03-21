#ifndef CARTESIAN_RADIAL_POTENTIAL_FIELDS_NODE_HPP
#define CARTESIAN_RADIAL_POTENTIAL_FIELDS_NODE_HPP

#include <wbc_msgs/msg/rigid_body_state.hpp>
#include <wbc_msgs/msg/radial_potential_field_vector.hpp>

#include "controller_node.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>
#include <wbc/controllers/CartesianPotentialFieldsController.hpp>
#include <wbc/controllers/PotentialField.hpp>

/**
@brief Radial potential fields in Cartesian space. See <a href="https://github.com/ARC-OPT/wbc/blob/master/src/controllers/CartesianPotentialFieldsController.hpp">here</a> for details.
*/
class CartesianRadialPotentialFieldsNode : public ControllerNode{
protected:
    wbc_msgs::msg::RigidBodyState control_output_msg;

    ctrl_lib::CartesianPotentialFieldsController* controller;
    double influence_distance;
    base::samples::RigidBodyStateSE3 feedback;
    base::samples::RigidBodyStateSE3 control_output;
    std::vector<ctrl_lib::PotentialFieldPtr> fields;

    rclcpp::Subscription<wbc_msgs::msg::RadialPotentialFieldVector>::SharedPtr sub_setpoint;
    rclcpp::Subscription<wbc_msgs::msg::RigidBodyState>::SharedPtr sub_feedback;
    rclcpp::Publisher<wbc_msgs::msg::RigidBodyState>::SharedPtr control_output_publisher;

public:
    CartesianRadialPotentialFieldsNode(const std::string& node_name);
    ~CartesianRadialPotentialFieldsNode();

    void feedbackCallback(const wbc_msgs::msg::RigidBodyState& msg);
    void potFieldsCallback(const wbc_msgs::msg::RadialPotentialFieldVector& msg);
    virtual void updateController();
};

#endif
