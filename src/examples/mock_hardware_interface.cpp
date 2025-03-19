#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_control_msgs/msg/robot_state.hpp>
#include <robot_control_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std;

namespace wbc_ros{

class MockHardwareInterface  : public rclcpp_lifecycle::LifecycleNode{

    using CommandMsg = robot_control_msgs::msg::JointCommand;
    using CommandMsgPtr = std::shared_ptr<CommandMsg>;
    using CommandSubscription = rclcpp::Subscription<CommandMsg>::SharedPtr;

    using JointStateMsg = sensor_msgs::msg::JointState;
    using JointStatePublisher = rclcpp::Publisher<JointStateMsg>::SharedPtr;

    using RobotStateMsg = robot_control_msgs::msg::RobotState;
    using RobotStatePublisher = rclcpp::Publisher<RobotStateMsg>::SharedPtr;

public:
   MockHardwareInterface(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("mock_hardware_interface", options){

        robot_state_publisher = this->create_publisher<RobotStateMsg>("~/robot_state", 10);
        joint_state_publisher = this->create_publisher<JointStateMsg>("/joint_states", 10);
        command_subscriber = this->create_subscription<CommandMsg>("~/command", rclcpp::SystemDefaultsQoS(), bind(&MockHardwareInterface::command_callback, this, placeholders::_1));

        this->declare_parameter("update_rate", 0);
        this->declare_parameter("joint_names", vector<string>());
        this->declare_parameter("initial_position", vector<double>());
        this->declare_parameter("contacts", vector<bool>());

        update_rate = this->get_parameter("update_rate").as_int(); 
        vector<string> joint_names = this->get_parameter("joint_names").as_string_array(); 
        vector<double> initial_position = this->get_parameter("initial_position").as_double_array(); 
        vector<bool> contacts = this->get_parameter("contacts").as_bool_array(); 

        robot_state.joint_state.position.resize(joint_names.size());
        robot_state.joint_state.velocity.resize(joint_names.size());
        robot_state.joint_state.acceleration.resize(joint_names.size());      
        for(auto c : contacts)
            robot_state.contacts.active.push_back(c);

        joint_command.position.resize(joint_names.size());
        joint_command.velocity.resize(joint_names.size());
        joint_command.effort.resize(joint_names.size());
        for(uint i = 0; i < joint_names.size(); i++){
            joint_command.position[i] = initial_position[i];
            joint_command.velocity[i] = 0.0;
            joint_command.effort[i] = 0.0;
        }

        joint_state.name = joint_names;
        joint_state.position.resize(joint_names.size());
        joint_state.velocity.resize(joint_names.size());
        joint_state.effort.resize(joint_names.size());

        timer = this->create_wall_timer(1ms, std::bind(&MockHardwareInterface::timer_callback, this));
        timer->cancel();
    }

private:
    void timer_callback(){
        joint_state.position = joint_command.position;
        joint_state.velocity = joint_command.velocity;
        joint_state.effort   = joint_command.effort;
        joint_state.header.stamp = this->get_clock()->now();
        joint_state_publisher->publish(joint_state);

        robot_state.joint_state.position = joint_state.position;
        robot_state.joint_state.velocity = joint_state.velocity;
        robot_state.joint_state.effort = joint_state.effort;
        robot_state_publisher->publish(robot_state);
    }
    void command_callback(const robot_control_msgs::msg::JointCommand::SharedPtr msg){
        joint_command = *msg;
    }
    virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
        timer->reset();
        RCLCPP_INFO(this->get_logger(), "Mock Hardware Interface is running at %i Hz", update_rate);
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
        timer->cancel();
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }
    virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }
    virtual CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/){
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }
    virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }

    CommandMsg joint_command;
    RobotStateMsg robot_state;
    JointStateMsg joint_state;
    CommandMsg command;

    rclcpp::TimerBase::SharedPtr timer;
    int update_rate;

    CommandSubscription command_subscriber;
    RobotStatePublisher robot_state_publisher;
    JointStatePublisher joint_state_publisher;   

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(wbc_ros::MockHardwareInterface)
