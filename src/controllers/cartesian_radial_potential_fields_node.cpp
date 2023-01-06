#include "cartesian_radial_potential_fields_node.hpp"
#include <wbc/controllers/RadialPotentialField.hpp>
#include "../conversions.hpp"

using namespace ctrl_lib;

CartesianRadialPotentialFieldsNode::CartesianRadialPotentialFieldsNode(int argc, char** argv) : ControllerNode(argc, argv){

    controller = new CartesianPotentialFieldsController();

    checkParam("influence_distance");
    ros::param::get("influence_distance", influence_distance);

    checkParam("p_gain");
    std::vector<double> p_gain;
    ros::param::get("p_gain", p_gain);
    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));

    checkParam("max_control_output");
    std::vector<double> max_control_output;
    ros::param::get("max_control_output", max_control_output);
    controller->setMaxControlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));

    // Potential Fields
    sub_setpoint = nh->subscribe("pot_field_centers", 1, &CartesianRadialPotentialFieldsNode::potFieldsCallback, this);
    // controller feedback
    sub_feedback = nh->subscribe("feedback", 1, &CartesianRadialPotentialFieldsNode::feedbackCallback, this);
    // Ctrl output
    control_output_publisher = nh->advertise<wbc_msgs::RigidBodyState>("control_output", 1);
}

CartesianRadialPotentialFieldsNode::~CartesianRadialPotentialFieldsNode(){
    delete controller;
}

void CartesianRadialPotentialFieldsNode::feedbackCallback(const wbc_msgs::RigidBodyState& msg){
    fromROS(msg, feedback);
    has_feedback = true;
}

void CartesianRadialPotentialFieldsNode::potFieldsCallback(const wbc_msgs::RadialPotentialFieldVector& msg){
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
    control_output_publisher.publish(control_output_msg);
}

int main(int argc, char** argv){
    CartesianRadialPotentialFieldsNode node(argc, argv);
    node.run();
    return 0;
}
