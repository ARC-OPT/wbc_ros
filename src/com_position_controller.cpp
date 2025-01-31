#include <wbc_ros/com_position_controller.hpp>
#include <wbc_ros/conversions.hpp>
#include "pluginlib/class_list_macros.hpp"
#include <wbc_ros/defines.hpp>

using namespace wbc;
using namespace std;
using namespace rclcpp;
using namespace hardware_interface;

namespace wbc_ros{

CoMPositionController::CoMPositionController() : ChainableControllerInterface(){
}

void CoMPositionController::setpoint_callback(const RbsMsgPtr msg){
    rt_setpoint_buffer.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration CoMPositionController::command_interface_configuration() const{
    /*
    Order of command interfaces:
      twist(vx,vy.vz) - for ControlMode::velocity
      spatial acc.(vx_dot,vy_dot,vz_dot)  - for ControlMode::acceleration
    */
    vector<string> iface_names;
    string prefix = params.wbc_name + "/" + params.task_name + "/reference/";
    if(params.control_mode == ControlMode::velocity)
        iface_names = prefix + vel_interfaces;
    else
        iface_names = prefix + acc_interfaces;
    return { controller_interface::interface_configuration_type::INDIVIDUAL, iface_names};
}

controller_interface::InterfaceConfiguration CoMPositionController::state_interface_configuration() const{
    /*
    Order of state interfaces (n=13):
    position(x,y,z), velocity(vx,vy,vz)
    */
    controller_interface::InterfaceConfiguration conf;
    string prefix = params.wbc_name + "/" + params.task_name + "/status/";
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    if(params.control_mode == ControlMode::velocity)
        conf.names = prefix + (pos_interfaces);
    else
        conf.names = prefix + (pos_interfaces + vel_interfaces);
    return conf;
}

vector<hardware_interface::CommandInterface> CoMPositionController::on_export_reference_interfaces(){
    /*
    Order of reference interfaces:
    position(x,y,z), velocity(vx,vy,vz), acceleration(vx_dot,vy_dot,vz_dot)
    */
    vector<string> reference_interface_names;
    if(params.control_mode == ControlMode::velocity)
        reference_interface_names = pos_interfaces + vel_interfaces;
    else
        reference_interface_names = pos_interfaces + vel_interfaces + acc_interfaces;
    reference_interfaces_.resize(reference_interface_names.size());
    reference_interface_values.resize(reference_interface_names.size());
    vector<hardware_interface::CommandInterface> interfaces;
    string node_name = get_node()->get_name();
    uint idx = 0;
    for(const string& s : reference_interface_names)
        interfaces.push_back(CommandInterface(node_name, "setpoint/" + s, reference_interface_values.data() + idx++));
    return interfaces;
}

controller_interface::return_type CoMPositionController::update_reference_from_subscribers(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    setpoint_msg = *rt_setpoint_buffer.readFromRT();
    if(!setpoint_msg.get())
        return controller_interface::return_type::OK;
    toRaw(*setpoint_msg, reference_interface_values);
    return controller_interface::return_type::OK;
}

bool CoMPositionController::read_feedback(){
    state_interface_values.resize(state_interfaces_.size());
    bool success = true;
    for(uint i = 0; i < state_interfaces_.size(); i++)
        success &= state_interfaces_[i].get_value(state_interface_values[i]);
    if(params.control_mode == ControlMode::velocity)
        feedback.pose.position = state_interface_values;
    else{
        feedback.pose.position = state_interface_values.segment(0,3);
        feedback.twist.linear  = state_interface_values.segment(3,3);
    }
    return success;
}

bool CoMPositionController::read_setpoint(){

    for(uint i = 0; i < reference_interface_values.size(); i++){
        if(isnan(reference_interface_values[i]))
            return false;
    }
    if(params.control_mode == ControlMode::velocity)
        fromRaw(reference_interface_values, setpoint.pose, setpoint.twist);
    else
        fromRaw(reference_interface_values, setpoint.pose, setpoint.twist, setpoint.acceleration);

    return true;
}

void CoMPositionController::write_control_output(){
    bool success = true;
    for(uint i = 0; i < 3; i++){
        if(params.control_mode == ControlMode::velocity)
            success &= command_interfaces_[i].set_value(control_output.twist.linear[i]);
        else
            success &= command_interfaces_[i].set_value(control_output.acceleration.linear[i]);
    }
    // TODO: What to do with the return value? 
    if(!success)
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set value of some command interface");
}

controller_interface::CallbackReturn CoMPositionController::on_init(){
    try{
        // Create the parameter listener and read all ROS parameters
        param_listener = make_shared<com_position_controller::ParamListener>(get_node());
        params = param_listener->get_params();
    }
    catch (const exception & e){
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoMPositionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

    if(params.control_mode != ControlMode::velocity && params.control_mode != ControlMode::acceleration){
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid control mode or control mode not set: %i", (int)params.control_mode);
        return CallbackReturn::ERROR;
    }

    controller = new CartesianPosPDController();
    Eigen::VectorXd p_gain(6), d_gain(6), max_control_output(6);
    p_gain.setZero();
    d_gain.setZero();
    max_control_output.setConstant(1e8);
    p_gain.segment(0,3) = Eigen::Map<Eigen::VectorXd>(params.p_gain.data(),params.p_gain.size());
    d_gain.segment(0,3) = Eigen::Map<Eigen::VectorXd>(params.d_gain.data(),params.d_gain.size());
    max_control_output.segment(0,3) = Eigen::Map<Eigen::VectorXd>(params.max_control_output.data(),params.max_control_output.size());
    controller->setPGain(p_gain);
    controller->setDGain(d_gain);
    controller->setMaxCtrlOutput(max_control_output);

    feedback.pose.orientation.setIdentity();
    feedback.twist.angular.setZero();
    setpoint.pose.orientation.setIdentity();
    setpoint.twist.angular.setZero();
    setpoint.acceleration.angular.setZero();

    setpoint_subscriber = get_node()->create_subscription<RbsMsg>("~/setpoint",
        rclcpp::SystemDefaultsQoS(), bind(&CoMPositionController::setpoint_callback, this, placeholders::_1));
    control_output_publisher = get_node()->create_publisher<RbsMsg>("~/control_output", rclcpp::SystemDefaultsQoS());
    //rt_control_output_publisher = make_unique<RTRbsPublisher>(control_output_publisher);
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoMPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    for(uint i = 0; i < reference_interface_values.size(); i++)
        reference_interface_values[i] = numeric_limits<double>::quiet_NaN();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type CoMPositionController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    if(!read_feedback()){
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "%s: No feedback", get_node()->get_name());
        return controller_interface::return_type::OK;
    }
    if(!read_setpoint()){
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "%s: No setpoint", get_node()->get_name());
        return controller_interface::return_type::OK;
    }

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
    write_control_output();

    /*rt_control_output_publisher->lock();
    toROS(control_output, rt_control_output_publisher->msg_);
    rt_control_output_publisher->unlockAndPublish();*/

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CoMPositionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoMPositionController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    delete controller;
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoMPositionController::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CoMPositionController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

}

PLUGINLIB_EXPORT_CLASS(wbc_ros::CoMPositionController, controller_interface::ChainableControllerInterface)
