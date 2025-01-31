#include <wbc_ros/cartesian_position_controller.hpp>
#include <wbc_ros/conversions.hpp>

#include <rclcpp_components/register_node_macro.hpp>

using namespace wbc;
using namespace rclcpp;

namespace wbc_ros{

CartesianPositionController::CartesianPositionController(const NodeOptions & options) : LifecycleNode("cartesian_position_controller", options){
  
    // Create the parameter listener and read all ROS parameters
    param_listener = std::make_shared<cartesian_position_controller::ParamListener>(this->get_node_parameters_interface());
    params = param_listener->get_params();

}

void CartesianPositionController::setpoint_callback(const RbsMsgPtr msg){
    rt_setpoint_buffer.writeFromNonRT(msg);
    has_setpoint = true;
}
void CartesianPositionController::feedback_callback(const RbsMsgPtr msg){
    rt_feedback_buffer.writeFromNonRT(msg);
    has_feedback = true;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn CartesianPositionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    controller = new CartesianPosPDController();
    controller->setPGain(Eigen::Map<Eigen::VectorXd>(params.p_gain.data(),params.p_gain.size()));
    controller->setDGain(Eigen::Map<Eigen::VectorXd>(params.d_gain.data(),params.d_gain.size()));
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(params.max_control_output.data(),params.max_control_output.size()));

    if(params.control_mode != ControlMode::velocity && params.control_mode != ControlMode::acceleration){
        RCLCPP_ERROR(this->get_logger(), "Invalid control mode or control mode not set: %i", (int)params.control_mode);
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }

    setpoint_subscriber = this->create_subscription<RbsMsg>("~/setpoint", 
        SystemDefaultsQoS(), bind(&CartesianPositionController::setpoint_callback, this, std::placeholders::_1));
    feedback_subscriber = this->create_subscription<RbsMsg>(params.wbc_name + "/" + params.task_name + "/status", 
        SystemDefaultsQoS(), bind(&CartesianPositionController::feedback_callback, this, std::placeholders::_1));
    control_output_publisher = this->create_publisher<RbsMsg>(params.wbc_name + "/" + params.task_name + "/reference", SystemDefaultsQoS());
    rt_control_output_publisher = std::make_unique<RTRbsPublisher>(control_output_publisher);

    timer = this->create_wall_timer(std::chrono::milliseconds((int)1000.0/params.update_rate),std::bind(&CartesianPositionController::update, this));
    timer->cancel(); 

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn CartesianPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    timer->reset(); 
    has_setpoint = has_feedback = false;
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void CartesianPositionController::update(){
    if(!has_feedback){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s: No feedback", this->get_name());
        return;
    }
    feedback_msg = *rt_feedback_buffer.readFromRT();
    if(feedback_msg.get())
        fromROS(*feedback_msg, feedback);

    if(!has_setpoint){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s: No setpoint", this->get_name());
        return;
    }
    setpoint_msg = *rt_setpoint_buffer.readFromRT();
    if(setpoint_msg.get())
        fromROS(*setpoint_msg, setpoint);


    if(params.control_mode == ControlMode::velocity)
        control_output.twist = controller->update(setpoint.pose, 
                                                  setpoint.twist, 
                                                  feedback.pose);
    else
        control_output.acceleration = controller->update(setpoint.pose, 
                                                         setpoint.twist, 
                                                         setpoint.acceleration, 
                                                         feedback.pose, 
                                                         feedback.twist);   

    std::cout<<"Setpoint"<<std::endl;
    std::cout<<setpoint.pose.position.transpose()<<std::endl;
    std::cout<<"Feedback"<<std::endl;
    std::cout<<feedback.pose.position.transpose()<<std::endl;
    std::cout<<"Control output"<<std::endl;
    std::cout<<control_output.twist.linear.transpose()<<std::endl<<std::endl;

    rt_control_output_publisher->lock();
    toROS(control_output, rt_control_output_publisher->msg_);
    rt_control_output_publisher->unlockAndPublish();
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn CartesianPositionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    timer->cancel(); 
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn CartesianPositionController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    delete controller;
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn CartesianPositionController::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn CartesianPositionController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(wbc_ros::CartesianPositionController)
