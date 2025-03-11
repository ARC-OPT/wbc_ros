#include "mock_hardware_interface.hpp"
#include <rclcpp_components/register_node_macro.hpp>

using namespace std;

namespace wbc_ros{

MockHardwareInterface::MockHardwareInterface(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("mock_hardware_interface", options){
}

void MockHardwareInterface::command_callback(const CommandMsgPtr msg){
    joint_command = *msg;
    has_command = true;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn MockHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    robot_state_publisher = this->create_publisher<robot_control_msgs::msg::RobotState>("~/robot_state", 10);
    joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    command_subscriber = this->create_subscription<CommandMsg>("~/command", rclcpp::SystemDefaultsQoS(), bind(&MockHardwareInterface::command_callback, this, placeholders::_1));
    
    this->declare_parameter("joint_names", vector<string>());
    this->declare_parameter("update_rate", 1000);
    this->declare_parameter("initial_position", vector<double>());
    
    vector<string> joint_names = this->get_parameter("joint_names").as_string_array(); 
    int update_rate = this->get_parameter("update_rate").as_int(); 
    vector<double> initial_position = this->get_parameter("initial_position").as_double_array(); 

    robot_state.joint_state.position.resize(joint_names.size());
    robot_state.joint_state.velocity.resize(joint_names.size());
    robot_state.joint_state.acceleration.resize(joint_names.size());

    joint_command.position.resize(joint_names.size());
    joint_command.velocity.resize(joint_names.size());
    for(uint i = 0; i < joint_names.size(); i++){
        joint_command.position[i] = initial_position[i];
        joint_command.velocity[i] = 0.0;
    }

    joint_state.name = joint_names;
    joint_state.position.resize(joint_names.size());
    joint_state.velocity.resize(joint_names.size());

    timer = this->create_wall_timer(std::chrono::milliseconds((int)1000/update_rate),std::bind(&MockHardwareInterface::updateController, this));
    timer->cancel(); 

    has_command = false;

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void MockHardwareInterface::updateController(){
    joint_state.position = joint_command.position;
    joint_state.velocity = joint_command.velocity;
    joint_state.header.stamp = this->get_clock()->now();
    joint_state_publisher->publish(joint_state);

    robot_state.joint_state.position = joint_state.position;
    robot_state.joint_state.velocity = joint_state.velocity;
    robot_state_publisher->publish(robot_state);
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn MockHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    timer->reset(); 
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::LifecycleNode::CallbackReturn MockHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    timer->cancel(); 
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::LifecycleNode::CallbackReturn MockHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::LifecycleNode::CallbackReturn MockHardwareInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::LifecycleNode::CallbackReturn MockHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;

}

}

RCLCPP_COMPONENTS_REGISTER_NODE(wbc_ros::MockHardwareInterface)