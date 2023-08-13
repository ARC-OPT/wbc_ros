#ifndef WBC_ROS_CARTESIAN_POSITION_CONTROLLER_HPP
#define WBC_ROS_CARTESIAN_POSITION_CONTROLLER_HPP

#include <wbc_msgs/msg/rigid_body_state.hpp>

#include <controller_interface/chainable_controller_interface.hpp>
#include <wbc/controllers/CartesianPosPDController.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc_ros{

/**
@brief Position controller in Cartesian space. See <a href="https://github.com/ARC-OPT/wbc/blob/master/src/controllers/CartesianPosPDController.hpp">here</a> for details.
*/
class CartesianPositionController : public controller_interface::ChainableControllerInterface{
protected:
    wbc_msgs::msg::RigidBodyState control_output_msg;

    ctrl_lib::CartesianPosPDController* controller;
    base::samples::RigidBodyStateSE3 feedback;
    base::samples::RigidBodyStateSE3 setpoint;
    base::samples::RigidBodyStateSE3 control_output;

    // rclcpp::Subscription<wbc_msgs::msg::RigidBodyState>::SharedPtr sub_setpoint;
    // rclcpp::Subscription<wbc_msgs::msg::RigidBodyState>::SharedPtr sub_feedback;
    // rclcpp::Publisher<wbc_msgs::msg::RigidBodyState>::SharedPtr control_output_publisher;

public:
    CartesianPositionController();
    ~CartesianPositionController();

    virtual controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    virtual controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    virtual controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    virtual controller_interface::CallbackReturn on_init() override;
    virtual controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    virtual controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    virtual controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    virtual controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    virtual controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
    virtual controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    virtual std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
    virtual controller_interface::return_type update_reference_from_subscribers();
    
    // void setpointCallback(const wbc_msgs::msg::RigidBodyState& msg);
    // void feedbackCallback(const wbc_msgs::msg::RigidBodyState& msg);
    // virtual void updateController();
};

}

#endif
