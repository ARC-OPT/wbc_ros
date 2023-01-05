#include "cartesian_radial_potential_fields_node.hpp"
#include <wbc/controllers/RadialPotentialField.hpp>
#include "../conversions.hpp"

using namespace ctrl_lib;

CartesianRadialPotentialFieldsNode::CartesianRadialPotentialFieldsNode(int argc, char** argv) : has_feedback(false), has_pot_fields(false){
    node_name = "cartesian_potential_fields";
    ros::init(argc, argv, node_name);
    nh = new ros::NodeHandle();

    ROS_INFO("Initialize Controller: %s", node_name.c_str());

    if(!ros::param::has("control_rate")){
        ROS_ERROR("WBC parameter control_rate has not been set");
        abort();
    }
    ros::param::get("control_rate", control_rate);

    controller = new CartesianPotentialFieldsController();

    if(!ros::param::has("influence_distance")){
        ROS_ERROR("WBC parameter influence_distance has not been set");
        abort();
    }
    ros::param::get("influence_distance", influence_distance);

    if(!ros::param::has("p_gain")){
        ROS_ERROR("WBC parameter p_gain has not been set");
        abort();
    }
    std::vector<double> p_gain;
    ros::param::get("p_gain", p_gain);
    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));

    if(!ros::param::has("max_control_output")){
        ROS_ERROR("WBC parameter max_control_output has not been set");
        abort();
    }
    std::vector<double> max_control_output;
    ros::param::get("max_control_output", max_control_output);
    controller->setMaxControlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));

    // Potential Fields
    sub_pot_fields = nh->subscribe("pot_field_centers", 1, &CartesianRadialPotentialFieldsNode::potFieldsCallback, this);
    // controller feedback
    sub_feedback = nh->subscribe("feedback", 1, &CartesianRadialPotentialFieldsNode::feedbackCallback, this);
    // Ctrl output
    control_output_publisher = nh->advertise<wbc_msgs::RigidBodyState>("control_output", 1);
}

CartesianRadialPotentialFieldsNode::~CartesianRadialPotentialFieldsNode(){
    delete controller;
    delete nh;
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
    has_pot_fields = true;
}

void CartesianRadialPotentialFieldsNode::update(){
    if(!has_feedback){
        ROS_WARN_DELAYED_THROTTLE(5, "%s: No feedback", node_name.c_str());
        return;
    }
    if(!has_pot_fields){
        ROS_DEBUG_DELAYED_THROTTLE(5, "%s: No potential fields", node_name.c_str());
        return;
    }
    controller->setFields(fields);
    control_output = controller->update(feedback);
    toROS(control_output, control_output_msg);
    control_output_publisher.publish(control_output_msg);
}

void CartesianRadialPotentialFieldsNode::run(){
    ros::Rate loop_rate(control_rate);
    ROS_INFO("Cartesian Radial Potential Fields Controller is running");
    while(ros::ok()){
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv){
    CartesianRadialPotentialFieldsNode node(argc, argv);
    node.run();
    return 0;
}
