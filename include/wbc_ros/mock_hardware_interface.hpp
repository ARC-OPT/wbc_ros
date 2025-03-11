#ifndef WBC_ROBOT_INTERFACE_HPP
#define WBC_ROBOT_INTERFACE_HPP


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_control_msgs/msg/robot_state.hpp>
#include <robot_control_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace wbc_ros{

class MockHardwareInterface : public rclcpp_lifecycle::LifecycleNode{

    using CommandMsg = robot_control_msgs::msg::JointCommand;
    using CommandMsgPtr = std::shared_ptr<CommandMsg>;
    using CommandSubscription = rclcpp::Subscription<CommandMsg>::SharedPtr;

    using JointStateMsg = sensor_msgs::msg::JointState;
    using JointStatePublisher = rclcpp::Publisher<JointStateMsg>::SharedPtr;

    using RobotStateMsg = robot_control_msgs::msg::RobotState;
    using RobotStatePublisher = rclcpp::Publisher<RobotStateMsg>::SharedPtr;

protected:
   CommandMsg joint_command;
   RobotStateMsg robot_state;
   JointStateMsg joint_state;
   CommandMsg command;

   rclcpp::TimerBase::SharedPtr timer;
   bool has_command;   
   
   CommandSubscription command_subscriber;
   RobotStatePublisher robot_state_publisher;

   JointStatePublisher joint_state_publisher;   

   void command_callback(const CommandMsgPtr msg);
   void updateController();

public:
   MockHardwareInterface(const rclcpp::NodeOptions & options);
   ~MockHardwareInterface(){}

   virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
   virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
   virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
   virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
   virtual CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
   virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
};

}

#endif
