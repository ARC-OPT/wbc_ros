#include "wbc_ros/cartesian_position_controller.hpp"
#include "conversions.hpp"
#include "pluginlib/class_list_macros.hpp"

using namespace ctrl_lib;
using namespace std;
using namespace rclcpp;
using namespace hardware_interface;

namespace wbc_ros{

CartesianPositionController::CartesianPositionController() : ChainableControllerInterface(){

    controller = new CartesianPosPDController();

    /*declare_parameter("p_gain", std::vector<double>());
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
    sub_setpoint = create_subscription<wbc_msgs::msg::RigidBodyState>("setpoint", 1, bind(&CartesianPositionController::setpointCallback, this, placeholders::_1));
    // controller feedback
    sub_feedback = create_subscription<wbc_msgs::msg::RigidBodyState>("feedback", 1, bind(&CartesianPositionController::feedbackCallback, this, placeholders::_1));
    // Ctrl output
    control_output_publisher = create_publisher<wbc_msgs::msg::RigidBodyState>("control_output", 1);*/
}

CartesianPositionController::~CartesianPositionController(){
    delete controller;
}

// void CartesianPositionController::setpointCallback(const wbc_msgs::msg::RigidBodyState& msg){
//     fromROS(msg, setpoint);
//     has_setpoint = true;
// }
//
// void CartesianPositionController::feedbackCallback(const wbc_msgs::msg::RigidBodyState& msg){
//     fromROS(msg, feedback);
//     has_feedback = true;
// }
//
// void CartesianPositionController::updateController(){
//     control_output = controller->update(setpoint, feedback);
//     toROS(control_output, control_output_msg);
//     control_output_publisher->publish(control_output_msg);
// }

controller_interface::InterfaceConfiguration CartesianPositionController::command_interface_configuration() const{
}

controller_interface::InterfaceConfiguration CartesianPositionController::state_interface_configuration() const{
}

controller_interface::return_type CartesianPositionController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period){
}

controller_interface::CallbackReturn CartesianPositionController::on_init(){
}

controller_interface::CallbackReturn CartesianPositionController::on_configure(const rclcpp_lifecycle::State & previous_state){
}

controller_interface::CallbackReturn CartesianPositionController::on_activate(const rclcpp_lifecycle::State & previous_state){
}

controller_interface::CallbackReturn CartesianPositionController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
}

controller_interface::CallbackReturn CartesianPositionController::on_cleanup(const rclcpp_lifecycle::State & previous_state){
}

controller_interface::CallbackReturn CartesianPositionController::on_error(const rclcpp_lifecycle::State & previous_state){
}

controller_interface::CallbackReturn CartesianPositionController::on_shutdown(const rclcpp_lifecycle::State & previous_state){
}

std::vector<hardware_interface::CommandInterface> CartesianPositionController::on_export_reference_interfaces(){
}

controller_interface::return_type CartesianPositionController::update_reference_from_subscribers(){
}

}

PLUGINLIB_EXPORT_CLASS(wbc_ros::CartesianPositionController, controller_interface::ChainableControllerInterface)
