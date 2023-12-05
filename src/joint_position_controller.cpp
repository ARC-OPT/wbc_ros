#include "wbc_ros/joint_position_controller.hpp"
#include "conversions.hpp"
#include "pluginlib/class_list_macros.hpp"

using namespace ctrl_lib;
using namespace std;
using namespace hardware_interface;

namespace wbc_ros{

JointPositionController::JointPositionController() : controller_interface::ChainableControllerInterface(), has_setpoint(false){
}

void JointPositionController::setpoint_callback(const JointCommandMsgPtr msg){
    rt_setpoint_buffer.writeFromNonRT(msg);
    has_setpoint = true;
}
//
// void JointPositionController::updateController(){
//     control_output = controller->update(setpoint, feedback);
//     toROS(control_output, control_output_msg);
//     control_output_publisher->publish(control_output_msg);
// }

std::vector<hardware_interface::CommandInterface> JointPositionController::on_export_reference_interfaces(){
    /*
    Order of reference interfaces:
    position,velocity,acceleration for each joint
    */
    vector<hardware_interface::CommandInterface> interfaces;
    string prefix = "setpoint/";
    uint idx = 0;
    for(const string &joint_name : params.joint_names){
        for(const string &iface_name : reference_interface_names)
            interfaces.push_back(hardware_interface::CommandInterface(string(get_node()->get_name()), prefix + joint_name + "/" + iface_name, reference_interfaces_.data() + idx++));
    }
    return interfaces;
}

controller_interface::return_type JointPositionController::update_reference_from_subscribers(){
    setpoint_msg = *rt_setpoint_buffer.readFromRT();
    if(!setpoint_msg.get())
        return controller_interface::return_type::OK;
    toRaw(*setpoint_msg, reference_interfaces_);
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration JointPositionController::command_interface_configuration() const{
    vector<string> iface_names;
    string prefix = params.wbc_name + "/" + params.task_name + "/reference/";
    if(params.control_mode == "velocity"){
        for(const string& s : params.joint_names)
            iface_names.push_back(prefix + s + "/velocity");
    }
    else{
        for(const string& s : params.joint_names)
            iface_names.push_back(prefix + s + "/acceleration");
    }
    return { controller_interface::interface_configuration_type::INDIVIDUAL, iface_names};
}

controller_interface::InterfaceConfiguration JointPositionController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::ALL;
    return conf;
}

void JointPositionController::read_state_from_hardware_interfaces(){
    for(uint i = 0; i < feedback.size(); i++){
        if(has_state_interface(HW_IF_POSITION))
            feedback[i].position = state_interfaces_[state_indices[HW_IF_POSITION][i]].get_value();
        if(has_state_interface(HW_IF_VELOCITY))
            feedback[i].speed = state_interfaces_[state_indices[HW_IF_VELOCITY][i]].get_value();
        if(has_state_interface(HW_IF_ACCELERATION))
            feedback[i].acceleration = state_interfaces_[state_indices[HW_IF_ACCELERATION][i]].get_value();
        if(has_state_interface(HW_IF_EFFORT))
            feedback[i].effort = state_interfaces_[state_indices[HW_IF_EFFORT][i]].get_value();
    }
    feedback.time = base::Time::now();

}

controller_interface::CallbackReturn JointPositionController::on_init(){
    try{
        // Create the parameter listener and read all ROS parameters
        param_listener = make_shared<joint_position_controller::ParamListener>(get_node());
        params = param_listener->get_params();
    }
    catch (const exception & e){
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    controller = new JointPosPDController(params.joint_names);
    controller->setPGain(Eigen::Map<Eigen::VectorXd>(params.p_gain.data(),params.p_gain.size()));
    controller->setDGain(Eigen::Map<Eigen::VectorXd>(params.d_gain.data(),params.d_gain.size()));
    controller->setFFGain(Eigen::Map<Eigen::VectorXd>(params.ff_gain.data(),params.ff_gain.size()));
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(params.max_control_output.data(),params.max_control_output.size()));
    controller->setDeadZone(Eigen::Map<Eigen::VectorXd>(params.dead_zone.data(),params.dead_zone.size()));

    setpoint_subscriber = get_node()->create_subscription<JointCommandMsg>("~/setpoint",
        rclcpp::SystemDefaultsQoS(), bind(&JointPositionController::setpoint_callback, this, placeholders::_1));
    control_output_publisher = get_node()->create_publisher<JointCommandMsg>("~/control_output", rclcpp::SystemDefaultsQoS());
    rt_control_output_publisher = make_unique<RTJointCommandPublisher>(control_output_publisher);

    uint n = params.joint_names.size();
    feedback.resize(n);
    feedback.names = params.joint_names;

    setpoint.resize(n);
    setpoint.names = params.joint_names;

    rt_control_output_publisher->msg_.points.resize(1);
    rt_control_output_publisher->msg_.points[0].positions.resize(n);
    rt_control_output_publisher->msg_.points[0].velocities.resize(n);
    rt_control_output_publisher->msg_.points[0].accelerations.resize(n);
    rt_control_output_publisher->msg_.joint_names = params.joint_names;

    reference_interfaces_.resize(n*3);

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPositionController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    if(!has_setpoint){
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "%s: No setpoint", get_node()->get_name());
        return controller_interface::return_type::OK;
    }

    read_state_from_hardware_interfaces();

    fromRaw(reference_interfaces_, setpoint);
    control_output = controller->update(setpoint, feedback);

    write_control_output_to_hardware();

    rt_control_output_publisher->lock();
    toROS(control_output, rt_control_output_publisher->msg_);
    rt_control_output_publisher->unlockAndPublish();

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn JointPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    // Create state index maps here, since we need the interfaces to be configured first
    state_indices.clear();
    for(const std::string &joint_name : params.joint_names){
        for(const std::string &iface_name : allowed_state_interface_types){
            if(get_state_idx(joint_name, iface_name) != -1)
                state_indices[iface_name].push_back(get_state_idx(joint_name, iface_name));
        }
    }
    has_setpoint = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    delete controller;
        return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

void JointPositionController::write_control_output_to_hardware(){
    uint idx = 0;
    for(const string &s : params.joint_names){
        if(params.control_mode == "velocity")
            command_interfaces_[idx++].set_value(control_output[s].speed);
        else
            command_interfaces_[idx++].set_value(control_output[s].acceleration);
    }
}
}

PLUGINLIB_EXPORT_CLASS(wbc_ros::JointPositionController, controller_interface::ChainableControllerInterface)
