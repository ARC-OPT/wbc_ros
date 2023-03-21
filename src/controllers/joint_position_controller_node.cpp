#include "joint_position_controller_node.hpp"
#include "../conversions.hpp"

using namespace ctrl_lib;
using namespace std;
using namespace rclcpp;

JointPositionControllerNode::JointPositionControllerNode(const string& node_name) : ControllerNode(node_name){

    declare_parameter("joint_names", std::vector<string>());
    joint_names = get_parameter("joint_names").as_string_array();

    controller = new JointPosPDController(joint_names);

    declare_parameter("p_gain", std::vector<double>());
    declare_parameter("d_gain", std::vector<double>());
    declare_parameter("ff_gain", std::vector<double>());
    declare_parameter("max_control_output", std::vector<double>());
    declare_parameter("dead_zone", std::vector<double>());

    vector<double> p_gain = get_parameter("p_gain").as_double_array();
    vector<double> d_gain = get_parameter("p_gain").as_double_array();
    vector<double> ff_gain = get_parameter("p_gain").as_double_array();
    vector<double> max_control_output = get_parameter("p_gain").as_double_array();
    vector<double> dead_zone = get_parameter("p_gain").as_double_array();

    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));
    controller->setDGain(Eigen::Map<Eigen::VectorXd>(d_gain.data(),d_gain.size()));
    controller->setFFGain(Eigen::Map<Eigen::VectorXd>(ff_gain.data(),ff_gain.size()));
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));
    controller->setDeadZone(Eigen::Map<Eigen::VectorXd>(dead_zone.data(),dead_zone.size()));

    // controller setpoint
    sub_setpoint = create_subscription<trajectory_msgs::msg::JointTrajectory>("setpoint", 1, bind(&JointPositionControllerNode::setpointCallback, this, placeholders::_1));
    // controller feedback
    sub_feedback = create_subscription<sensor_msgs::msg::JointState>("feedback", 1, bind(&JointPositionControllerNode::feedbackCallback, this, placeholders::_1));
    // Ctrl output
    control_output_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("control_output", 1);
}

JointPositionControllerNode::~JointPositionControllerNode(){
    delete controller;
}

void JointPositionControllerNode::setpointCallback(const trajectory_msgs::msg::JointTrajectory& msg){
    has_setpoint = true;
    fromROS(msg, setpoint);
}

void JointPositionControllerNode::feedbackCallback(const sensor_msgs::msg::JointState& msg){
    has_feedback = true;
    fromROS(msg, feedback);
}

void JointPositionControllerNode::updateController(){
    control_output = controller->update(setpoint, feedback);
    toROS(control_output, control_output_msg);
    control_output_publisher->publish(control_output_msg);
}

int main(int argc, char** argv){
    init(argc, argv);
    spin(make_shared<JointPositionControllerNode>("joint_position_controller"));
    return 0;
}
