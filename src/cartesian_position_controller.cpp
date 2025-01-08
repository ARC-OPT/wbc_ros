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

controller_interface::InterfaceConfiguration CartesianPositionController::command_interface_configuration() const{
    /*
    Order of command interfaces:
    For velocity WBC:     twist(vx,vy.vz,wx,wy,wz)
    For acceleration WBC: spatial acc.(vx_dot,vy_dot,vz_dot,wx_dot,wy_dot,wz_dot)
    */
    vector<string> iface_names;
    string prefix = params.wbc_name + "/" + params.task_name + "/reference/";
    if(params.control_mode == "velocity"){
        for(const string& s : twist_interfaces)
            iface_names.push_back(prefix + s);
    }
    else{
        for(const string& s : acc_interfaces)
            iface_names.push_back(prefix + s);
    }
    return { controller_interface::interface_configuration_type::INDIVIDUAL, iface_names};
}

controller_interface::InterfaceConfiguration CartesianPositionController::state_interface_configuration() const{
    /*
    Order of state interfaces:
    position(x,y,z), orientation(qw,qx,qy,qz), twist(vx,vy.vz,wx,wy,wz)
    */
    controller_interface::InterfaceConfiguration conf;
    string prefix = params.wbc_name + "/" + params.task_name + "/status/";
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    vector<string> iface_names;
    for(const string& s : pose_interfaces)
        iface_names.push_back(prefix + s);
    conf.names = iface_names;

    return conf;
}

vector<hardware_interface::CommandInterface> CartesianPositionController::on_export_reference_interfaces(){
    /*
    Order of reference interfaces:
    position(x,y,z), orientation(qw,qx,qy,qz), twist(vx,vy.vz,wx,wy,wz), spatial acc.(vx_dot,vy_dot,vz_dot,wx_dot,wy_dot,wz_dot)
    */
    reference_interfaces_.resize(pose_interfaces.size()+twist_interfaces.size()+acc_interfaces.size());
    vector<hardware_interface::CommandInterface> interfaces;
    string node_name = get_node()->get_name();
    uint idx = 0;
    for(const string& s : pose_interfaces)
        interfaces.push_back(CommandInterface(node_name, "setpoint/" + s, reference_interfaces_.data() + idx++));
    for(const string& s : twist_interfaces)
        interfaces.push_back(CommandInterface(node_name, "setpoint/" + s, reference_interfaces_.data() + idx++));
    for(const string& s : acc_interfaces)
        interfaces.push_back(CommandInterface(node_name, "setpoint/" + s, reference_interfaces_.data() + idx++));
    return interfaces;
}

controller_interface::return_type CartesianPositionController::update_reference_from_subscribers(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    setpoint_msg = *rt_setpoint_buffer.readFromRT();
    if(!setpoint_msg.get())
        return controller_interface::return_type::OK;
    toRaw(*setpoint_msg, reference_interfaces_);
    return controller_interface::return_type::OK;
}

void CartesianPositionController::read_from_state_interfaces(){
    double val;
    if(params.control_mode == "velocity" || params.control_mode == "acceleration"){
        for(uint i = 0; i < 3; i++){
            assert(state_interfaces_[i].get_value(val));
            if(isnan(val)) return;
            feedback.pose.position[i] =  val;
        }
        for(uint i = 0; i < 4; i++){
            assert(state_interfaces_[i+3].get_value(val));
            if(isnan(val)) return;
            feedback.pose.orientation.coeffs()[i] =  val;
        }
    }
    if(params.control_mode == "acceleration"){
        for(uint i = 0; i < 3; i++){
            assert(state_interfaces_[i+7].get_value(val));
            if(isnan(val)) return;
            feedback.twist.linear[i] =  val;
        }
        for(uint i = 0; i < 3; i++){
            assert(state_interfaces_[i+10].get_value(val));
            if(isnan(val)) return;
            feedback.twist.angular[i] =  val;
        }
    }
    has_feedback = true;
}

void CartesianPositionController::write_control_output_to_hardware(){
    for(int i = 0; i < 3; i++){
        if(params.control_mode == "velocity"){
            assert(command_interfaces_[i].set_value(control_output.twist.linear[i]));
            assert(command_interfaces_[i+3].set_value(control_output.twist.angular[i]));
        }
        else if(params.control_mode == "acceleration"){
            assert(command_interfaces_[i].set_value(control_output.acceleration.linear[i]));
            assert(command_interfaces_[i+3].set_value(control_output.acceleration.angular[i]));
        }
        else
            assert("Invalid control mode");
    }
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
    controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(params.max_control_output.data(),params.max_control_output.size()));

    assert(params.control_mode == "velocity" || params.control_mode == "acceleration");
    
    setpoint_subscriber = get_node()->create_subscription<RbsMsg>("~/setpoint",
        rclcpp::SystemDefaultsQoS(), bind(&CartesianPositionController::setpoint_callback, this, placeholders::_1));
    control_output_publisher = get_node()->create_publisher<RbsMsg>("~/control_output", rclcpp::SystemDefaultsQoS());
    //rt_control_output_publisher = make_unique<RTRbsPublisher>(control_output_publisher);
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    has_feedback = has_setpoint = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPositionController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    read_from_state_interfaces();
    if(!has_feedback){
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "%s: No feedback", get_node()->get_name());
        return controller_interface::return_type::OK;
    }
    if(!has_setpoint){
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "%s: No setpoint", get_node()->get_name());
        return controller_interface::return_type::OK;
    }

    fromRaw(reference_interfaces_, setpoint);
    if(params.control_mode == "velocity"){
    	control_output.twist = controller->update(setpoint.pose, 
    	                                          setpoint.twist, 
    	                                          feedback.pose);
    }    	                                          
    else if(params.control_mode == "acceleration")
    	control_output.acceleration = controller->update(setpoint.pose, 
    	                                                 setpoint.twist,
                                                         setpoint.acceleration,
                                                         feedback.pose,
    	                                                 feedback.twist);
    else{
        RCLCPP_ERROR(get_node()->get_logger(), "%s: Invalid control mode: %s", get_node()->get_name(), params.control_mode.c_str());
        return controller_interface::return_type::ERROR;
    }

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

}

PLUGINLIB_EXPORT_CLASS(wbc_ros::CartesianPositionController, controller_interface::ChainableControllerInterface)
