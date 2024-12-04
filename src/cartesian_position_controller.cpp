#include "wbc_ros/cartesian_position_controller.hpp"
#include "conversions.hpp"
#include "pluginlib/class_list_macros.hpp"

using namespace wbc;
using namespace std;
using namespace rclcpp;
using namespace hardware_interface;

namespace wbc_ros{

CartesianPositionController::CartesianPositionController() : ChainableControllerInterface(), has_setpoint(false), has_feedback(false){
}

void CartesianPositionController::setpoint_callback(const RbsMsgPtr msg){
    rt_setpoint_buffer.writeFromNonRT(msg);
    has_setpoint = true;
}

void CartesianPositionController::feedback_callback(const RbsMsgPtr msg){
    rt_feedback_buffer.writeFromNonRT(msg);
    has_feedback = true;
}

controller_interface::InterfaceConfiguration CartesianPositionController::command_interface_configuration() const{
    vector<string> iface_names;
    string prefix = params.wbc_name + "/" + params.task_name + "/reference/";
    if(params.control_mode == "velocity"){
        for(const string& s : command_interfaces_vel)
            iface_names.push_back(prefix + s);
    }
    else{
        for(const string& s : command_interfaces_acc)
            iface_names.push_back(prefix + s);
    }
    return { controller_interface::interface_configuration_type::INDIVIDUAL, iface_names};
}

controller_interface::InterfaceConfiguration CartesianPositionController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    // We need Cartesian pose/twist as feedback. There seems to be no way to get feedback from the next controller in the chain
    // (which would be the WBC) and we don`t want to include an FK solver in this controller. Thus, we read the feedback from a topic (feedback_callback).
    conf.type = controller_interface::interface_configuration_type::NONE;
    return conf;
}

vector<hardware_interface::CommandInterface> CartesianPositionController::on_export_reference_interfaces(){
    /*
    Order of reference interfaces:
    position(x,y,z), orientation(qx,qy,qz,qw), twist(vx,vy.vz,wx,wy,wz), spatial acc.(vx_dot,vy_dot,vz_dot,wx_dot,wy_dot,wz_dot)
    */
    reference_interfaces_.resize(reference_interface_names.size());
    vector<hardware_interface::CommandInterface> interfaces;
    string prefix = "setpoint/";
    for(uint i = 0; i < reference_interface_names.size(); i++){
        const string s = reference_interface_names[i];
        interfaces.push_back(hardware_interface::CommandInterface(string(get_node()->get_name()), prefix + s, reference_interfaces_.data() + i));
    }
    return interfaces;
}

controller_interface::return_type CartesianPositionController::update_reference_from_subscribers(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    setpoint_msg = *rt_setpoint_buffer.readFromRT();
    if(!setpoint_msg.get())
        return controller_interface::return_type::OK;
    toRaw(*setpoint_msg, reference_interfaces_);
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CartesianPositionController::on_init(){
    try{
        // Create the parameter listener and read all ROS parameters
        param_listener = make_shared<cartesian_position_controller::ParamListener>(get_node());
        params = param_listener->get_params();
    }
    catch (const exception & e){
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    controller = new CartesianPosPDController();
    controller->setPGain(Eigen::Map<Eigen::VectorXd>(params.p_gain.data(),params.p_gain.size()));
    controller->setDGain(Eigen::Map<Eigen::VectorXd>(params.d_gain.data(),params.d_gain.size()));
    controller->setFFGain(Eigen::Map<Eigen::VectorXd>(params.ff_gain.data(),params.ff_gain.size()));
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(params.max_control_output.data(),params.max_control_output.size()));
    controller->setDeadZone(Eigen::Map<Eigen::VectorXd>(params.dead_zone.data(),params.dead_zone.size()));
    
    setpoint_subscriber = get_node()->create_subscription<RbsMsg>("~/setpoint",
        rclcpp::SystemDefaultsQoS(), bind(&CartesianPositionController::setpoint_callback, this, placeholders::_1));
    feedback_subscriber = get_node()->create_subscription<RbsMsg>(params.wbc_name + "/status_" + params.task_name,
        rclcpp::SystemDefaultsQoS(), bind(&CartesianPositionController::feedback_callback, this, placeholders::_1));
    control_output_publisher = get_node()->create_publisher<RbsMsg>("~/control_output", rclcpp::SystemDefaultsQoS());
    //rt_control_output_publisher = make_unique<RTRbsPublisher>(control_output_publisher);
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    has_feedback = has_setpoint = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPositionController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    if(!has_feedback){
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "%s: No feedback", get_node()->get_name());
        return controller_interface::return_type::OK;
    }
    if(!has_setpoint){
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "%s: No setpoint", get_node()->get_name());
        return controller_interface::return_type::OK;
    }

    feedback_msg = *rt_feedback_buffer.readFromRT();
    if (feedback_msg.get())
        fromROS(*feedback_msg, feedback);

    fromRaw(reference_interfaces_, setpoint);
    control_output = controller->update(setpoint, feedback);
    write_control_output_to_hardware();

    /*rt_control_output_publisher->lock();
    toROS(control_output, rt_control_output_publisher->msg_);
    rt_control_output_publisher->unlockAndPublish();*/

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CartesianPositionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    delete controller;
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionController::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

void CartesianPositionController::write_control_output_to_hardware(){
    for(int i = 0; i < 3; i++){
        if(params.control_mode == "velocity"){
            command_interfaces_[i].set_value(control_output.twist.linear[i]);
            command_interfaces_[i+3].set_value(control_output.twist.angular[i]);
        }
        else{
            command_interfaces_[i].set_value(control_output.acceleration.linear[i]);
            command_interfaces_[i+3].set_value(control_output.acceleration.angular[i]);
        }
    }
}

}

PLUGINLIB_EXPORT_CLASS(wbc_ros::CartesianPositionController, controller_interface::ChainableControllerInterface)
