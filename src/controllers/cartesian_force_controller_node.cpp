#include "cartesian_force_controller_node.hpp"
#include "../conversions.hpp"

using namespace ctrl_lib;
using namespace std;
using namespace rclcpp;

CartesianForceControllerNode::CartesianForceControllerNode(const string &node_name) : ControllerNode(node_name){

    controller = new CartesianForcePIDController();

    declare_parameter("p_gain", std::vector<double>());
    declare_parameter("i_gain", std::vector<double>());
    declare_parameter("d_gain", std::vector<double>());
    declare_parameter("windup", std::vector<double>());
    declare_parameter("max_control_output", std::vector<double>());
    declare_parameter("dead_zone", std::vector<double>());

    vector<double> p_gain = get_parameter("p_gain").as_double_array();
    vector<double> i_gain = get_parameter("i_gain").as_double_array();
    vector<double> d_gain = get_parameter("d_gain").as_double_array();
    vector<double> windup = get_parameter("windup").as_double_array();
    vector<double> max_control_output = get_parameter("max_control_output").as_double_array();
    vector<double> dead_zone = get_parameter("dead_zone").as_double_array();

    PIDCtrlParams pid_params;
    pid_params.p_gain = Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size());
    pid_params.i_gain = Eigen::Map<Eigen::VectorXd>(i_gain.data(),i_gain.size());
    pid_params.d_gain = Eigen::Map<Eigen::VectorXd>(d_gain.data(),d_gain.size());
    pid_params.windup = Eigen::Map<Eigen::VectorXd>(windup.data(),windup.size());
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));
    controller->setDeadZone(Eigen::Map<Eigen::VectorXd>(dead_zone.data(),dead_zone.size()));

    // controller setpoint
    sub_setpoint = create_subscription<geometry_msgs::msg::WrenchStamped>("setpoint", 1, bind(&CartesianForceControllerNode::setpointCallback, this, placeholders::_1));
    // controller feedback
    sub_feedback = create_subscription<geometry_msgs::msg::WrenchStamped>("feedback", 1, bind(&CartesianForceControllerNode::feedbackCallback, this, placeholders::_1));
    // Ctrl output
    control_output_publisher = create_publisher<wbc_msgs::msg::RigidBodyState>("control_output", 1);
}

CartesianForceControllerNode::~CartesianForceControllerNode(){
    delete controller;
}

void CartesianForceControllerNode::setpointCallback(const geometry_msgs::msg::WrenchStamped& msg){
    fromROS(msg, setpoint);
    has_setpoint = true;
}

void CartesianForceControllerNode::feedbackCallback(const geometry_msgs::msg::WrenchStamped& msg){
    fromROS(msg, feedback);
    has_feedback = true;
}

void CartesianForceControllerNode::updateController(){
    control_output = controller->update(setpoint, feedback, 1/control_rate);
    toROS(control_output, control_output_msg);
    control_output_publisher->publish(control_output_msg);
}

int main(int argc, char** argv){
    init(argc, argv);
    spin(make_shared<CartesianForceControllerNode>("cartesian_force_controller"));
    return 0;
}
