#include "cartesian_position_controller_node.hpp"
#include "../conversions.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace ctrl_lib;
using namespace std;
using namespace rclcpp;

namespace wbc_ros{

CartesianPositionControllerNode::CartesianPositionControllerNode(const rclcpp::NodeOptions &options) : ControllerNode("cartesian_position_controller", options){

    controller = new CartesianPosPDController();

    declare_parameter("p_gain", std::vector<double>());
    declare_parameter("d_gain", std::vector<double>());
    declare_parameter("ff_gain", std::vector<double>());
    declare_parameter("max_control_output", std::vector<double>());
    declare_parameter("dead_zone", std::vector<double>());

    vector<double> p_gain = get_parameter("p_gain").as_double_array();
    vector<double> d_gain = get_parameter("d_gain").as_double_array();
    vector<double> ff_gain = get_parameter("ff_gain").as_double_array();
    vector<double> max_control_output = get_parameter("max_control_output").as_double_array();
    vector<double> dead_zone = get_parameter("dead_zone").as_double_array();

    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));
    controller->setDGain(Eigen::Map<Eigen::VectorXd>(d_gain.data(),d_gain.size()));
    controller->setFFGain(Eigen::Map<Eigen::VectorXd>(ff_gain.data(),ff_gain.size()));
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));
    controller->setDeadZone(Eigen::Map<Eigen::VectorXd>(dead_zone.data(),dead_zone.size()));

    // controller setpoint
    sub_setpoint = create_subscription<wbc_msgs::msg::RigidBodyState>("setpoint", 1, bind(&CartesianPositionControllerNode::setpointCallback, this, placeholders::_1));
    // controller feedback
    sub_feedback = create_subscription<wbc_msgs::msg::RigidBodyState>("feedback", 1, bind(&CartesianPositionControllerNode::feedbackCallback, this, placeholders::_1));
    // Ctrl output
    control_output_publisher = create_publisher<wbc_msgs::msg::RigidBodyState>("control_output", 1);
}

CartesianPositionControllerNode::~CartesianPositionControllerNode(){
    delete controller;
}

void CartesianPositionControllerNode::setpointCallback(const wbc_msgs::msg::RigidBodyState& msg){
    fromROS(msg, setpoint);
    has_setpoint = true;
}

void CartesianPositionControllerNode::feedbackCallback(const wbc_msgs::msg::RigidBodyState& msg){
    fromROS(msg, feedback);
    has_feedback = true;
}

void CartesianPositionControllerNode::updateController(){
    control_output = controller->update(setpoint, feedback);
    toROS(control_output, control_output_msg);
    control_output_publisher->publish(control_output_msg);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(wbc_ros::CartesianPositionControllerNode)
