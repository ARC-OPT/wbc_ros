#include "cartesian_radial_potential_fields_node.hpp"
#include <wbc/controllers/RadialPotentialField.hpp>
#include "../conversions.hpp"

using namespace ctrl_lib;
using namespace std;
using namespace rclcpp;

CartesianRadialPotentialFieldsNode::CartesianRadialPotentialFieldsNode(const string& node_name) : ControllerNode(node_name){

    controller = new CartesianPotentialFieldsController();

    declare_parameter("p_gain", std::vector<double>());
    declare_parameter("influence_distance", 0.0);
    declare_parameter("max_control_output", std::vector<double>());

    vector<double> p_gain             = get_parameter("p_gain").as_double_array();
    influence_distance                = get_parameter("influence_distance").as_double();
    vector<double> max_control_output = get_parameter("max_control_output").as_double_array();

    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));
    controller->setMaxControlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));

    // Potential Fields
    sub_setpoint = create_subscription<wbc_msgs::msg::RadialPotentialFieldVector>("pot_field_centers", 1, bind(&CartesianRadialPotentialFieldsNode::potFieldsCallback, this, placeholders::_1));
    // controller feedback
    sub_feedback = create_subscription<wbc_msgs::msg::RigidBodyState>("feedback", 1, bind(&CartesianRadialPotentialFieldsNode::feedbackCallback, this, placeholders::_1));
    // Ctrl output
    control_output_publisher = create_publisher<wbc_msgs::msg::RigidBodyState>("control_output", 1);
}

CartesianRadialPotentialFieldsNode::~CartesianRadialPotentialFieldsNode(){
    delete controller;
}

void CartesianRadialPotentialFieldsNode::feedbackCallback(const wbc_msgs::msg::RigidBodyState& msg){
    fromROS(msg, feedback);
    has_feedback = true;
}

void CartesianRadialPotentialFieldsNode::potFieldsCallback(const wbc_msgs::msg::RadialPotentialFieldVector& msg){
    if(!has_feedback)
        return;
    fields.clear();
    for(size_t i = 0; i < msg.fields.size(); i++){
        if(msg.fields[i].header.frame_id == feedback.frame_id){
            PotentialFieldPtr pf = std::make_shared<RadialPotentialField>(3,msg.fields[i].id);
            pf->pot_field_center = base::Vector3d(msg.fields[i].position.x,
                                                  msg.fields[i].position.y,
                                                  msg.fields[i].position.z);
            pf->influence_distance = influence_distance;
            fields.push_back(pf);
        }
    }
    has_setpoint = true;
}

void CartesianRadialPotentialFieldsNode::updateController(){
    controller->setFields(fields);
    control_output = controller->update(feedback);
    toROS(control_output, control_output_msg);
    control_output_publisher->publish(control_output_msg);
}

int main(int argc, char** argv){
    init(argc, argv);
    spin(make_shared<CartesianRadialPotentialFieldsNode>("cartesian_radial_potential_fields_node"));
    return 0;
}
