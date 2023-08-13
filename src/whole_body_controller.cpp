#include "wbc_ros/whole_body_controller.hpp"
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/QPSolver.hpp>
#include "conversions.hpp"
#include "pluginlib/class_list_macros.hpp"

using namespace std;
using namespace wbc;
using namespace hardware_interface;

namespace wbc_ros{

WholeBodyController::WholeBodyController() : ChainableControllerInterface(), has_floating_base_state(false){

    /*

    // Input joint state
    joint_state.resize(robot_model->noOfJoints());
    joint_state.names = robot_model->jointNames();
    sub_feedback = create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, bind(&WholeBodyController::jointStateCallback, this, placeholders::_1));

    // Input floating base state
    if(robot_model_cfg.floating_base)
        sub_floating_base = create_subscription<wbc_msgs::msg::RigidBodyState>("floating_base_state", 1, bind(&WholeBodyController::floatingBaseStateCallback, this, placeholders::_1));

    // Input references, task weights and activations
    for(auto w : task_config){
        if(w.type == cart){
            function<void(const wbc_msgs::msg::RigidBodyState& msg)> fcn = bind(&WholeBodyController::cartReferenceCallback, this, placeholders::_1, w.name);
            sub_cart_ref.push_back(create_subscription<wbc_msgs::msg::RigidBodyState>("ref_" + w.name, 1, fcn));
        }
        else{
            function<void(const trajectory_msgs::msg::JointTrajectory& msg)> fcn = bind(&WholeBodyController::jntReferenceCallback, this, placeholders::_1, w.name);
            sub_jnt_ref.push_back(create_subscription<trajectory_msgs::msg::JointTrajectory>("ref_" + w.name, 1, fcn));
        }
        function<void(const std_msgs::msg::Float64MultiArray& msg)> fcn = bind(&WholeBodyController::taskWeightsCallback, this, placeholders::_1, w.name);
        sub_weights.push_back(create_subscription<std_msgs::msg::Float64MultiArray>("weights_" + w.name, 1, fcn));

        function<void(const std_msgs::msg::Float64& msg)> fcn2 = bind(&WholeBodyController::taskActivationCallback, this, placeholders::_1, w.name);
        sub_activation.push_back(create_subscription<std_msgs::msg::Float64>("activation_" + w.name, 1, fcn2));
    }

    // Output task states
    for(auto w : task_config){
        if(w.type == cart)
            publishers_task_status_cart.push_back(create_publisher<wbc_msgs::msg::RigidBodyState>("status_" + w.name, 1));
        else
            publishers_task_status_jnt.push_back(create_publisher<sensor_msgs::msg::JointState>("status_" + w.name, 1));
    }

    // Output task info
    for(auto w : task_config)
        publishers_task_info.push_back(create_publisher<wbc_msgs::msg::TaskStatus>("task_" + w.name, 1));

    // Input joint weights
    sub_joint_weights = create_subscription<std_msgs::msg::Float64MultiArray>("joint_weights", 1, bind(&WholeBodyController::jointWeightsCallback, this, placeholders::_1));

    // Stats on computation time
    pub_timing_stats = create_publisher<wbc_msgs::msg::WbcTimingStats>("timing_stats", 1);

    // Solver output
    solver_output_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("solver_output", 1);
    solver_output_raw_publisher = create_publisher<std_msgs::msg::Float64MultiArray>("solver_output_raw", 1);

    // WBC should send zeros if no setpoint is given from any controller, so set has_setpoint = true
    has_setpoint = true;
    stamp = get_clock()->now();

    RCLCPP_INFO(get_logger(), "WBC Node is running");*/
}

WholeBodyController::~WholeBodyController(){
}

controller_interface::InterfaceConfiguration WholeBodyController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::ALL;
    return conf;
}

controller_interface::InterfaceConfiguration WholeBodyController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::ALL;
    return conf;
}

void WholeBodyController::read_state_from_hardware(){
    for(uint i = 0; i < joint_state.size(); i++){
        if(has_state_interface(HW_IF_POSITION))
            joint_state[i].position = state_interfaces_[state_indices[HW_IF_POSITION][i]].get_value();
        if(has_state_interface(HW_IF_VELOCITY))
            joint_state[i].speed = state_interfaces_[state_indices[HW_IF_VELOCITY][i]].get_value();
        if(has_state_interface(HW_IF_ACCELERATION))
            joint_state[i].acceleration = state_interfaces_[state_indices[HW_IF_ACCELERATION][i]].get_value();
        if(has_state_interface(HW_IF_EFFORT))
            joint_state[i].effort = state_interfaces_[state_indices[HW_IF_EFFORT][i]].get_value();
    }
    joint_state.time = base::Time::now();
}

std::vector<hardware_interface::CommandInterface>WholeBodyController::on_export_reference_interfaces(){
    std::vector<hardware_interface::CommandInterface> chainable_command_interfaces;
    reference_interfaces_.resize(6);
    for(const auto &s : params.task_names){
        const std::string iface_names[6] = {s+"/vx", s+"/vy", s+"/vz", s+"/wx", s+"/wy", s+"/wz"};
        for(int i = 0; i < 6; i++)
            chainable_command_interfaces.push_back(hardware_interface::CommandInterface(std::string(get_node()->get_name()), iface_names[i], reference_interfaces_.data() + i));
    }
    return chainable_command_interfaces;
}

controller_interface::return_type WholeBodyController::update_reference_from_subscribers(){
    return controller_interface::return_type::OK;
}

controller_interface::return_type WholeBodyController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period){
    timing_stats.desired_period = 0;
    //timing_stats.actual_period = (get_node()->get_clock()->now() - stamp).seconds();
    stamp = get_node()->get_clock()->now();

    read_state_from_hardware();

    rclcpp::Time start = get_node()->get_clock()->now();
    robot_model->update(joint_state, floating_base_state);
    timing_stats.time_robot_model_update = (get_node()->get_clock()->now() - start).seconds();

    start = get_node()->get_clock()->now();
    qp = scene->update();
    timing_stats.time_scene_update = (get_node()->get_clock()->now() - start).seconds();

    start = get_node()->get_clock()->now();
    solver_output = scene->solve(qp);
    timing_stats.time_solve = (get_node()->get_clock()->now() - start).seconds();

    //tasks_status = scene->updateTasksStatus();

    if(integrate)
        joint_integrator.integrate(robot_model->jointState(robot_model->actuatedJointNames()), solver_output, 1.0/1.0);

    write_command_to_hardware();
    // publishTaskStatus();
    // publishTaskInfo();

    timing_stats.time_per_cycle = (get_node()->get_clock()->now() - stamp).seconds();
    timing_stats.header.stamp = get_node()->get_clock()->now();
    //pub_timing_stats->publish(timing_stats);
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn WholeBodyController::on_init(){
    try{
        // Create the parameter listener and read all ROS parameters
        param_listener = std::make_shared<whole_body_controller::ParamListener>(get_node());
        params = param_listener->get_params();
    }
    catch (const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_configure(const rclcpp_lifecycle::State & previous_state){

    integrate = params.integrate;

    RobotModelConfig robot_model_cfg;
    robot_model_cfg.file                 = params.robot_model.file;
    robot_model_cfg.type                 = params.robot_model.type;
    robot_model_cfg.submechanism_file    = params.robot_model.submechanism_file;
    robot_model_cfg.floating_base        = params.robot_model.floating_base;
    robot_model_cfg.contact_points.names = params.contact_names;
    robot_model_cfg.floating_base        = params.robot_model.floating_base;
    for(auto name : params.contact_names){
        const auto &c = params.robot_model.contact_points.contact_names_map.at(name);
        robot_model_cfg.contact_points.elements.push_back(ActiveContact(c.active, c.mu, c.wx, c.wy));
    }
    robot_model_cfg.validate();

    RCLCPP_INFO(get_node()->get_logger(), "Configuring robot model: %s", robot_model_cfg.type.c_str());
    PluginLoader::loadPlugin("libwbc-robot_models-" + robot_model_cfg.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(robot_model_cfg.type));
    if(!robot_model->configure(robot_model_cfg)){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure robot model");
        return CallbackReturn::ERROR;
    }

    QPSolverConfig solver_cfg(params.solver.type, params.solver.file);
    solver_cfg.validate();

    RCLCPP_INFO(get_node()->get_logger(), "Configuring solver: %s", solver_cfg.type.c_str());
    PluginLoader::loadPlugin("libwbc-solvers-" + solver_cfg.type + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(solver_cfg.type));

    task_config.clear();
    for(auto name : params.task_names){
        TaskConfig cfg;
        const auto& task_param = params.tasks.task_names_map.at(name);
        cfg.name        = name;
        cfg.priority    = task_param.priority;
        if(task_param.type == "cart")
            cfg.type = TaskType::cart;
        else if(task_param.type == "jnt")
            cfg.type = TaskType::jnt;
        else
            cfg.type = TaskType::com;
        cfg.weights     = task_param.weights;
        cfg.root        = task_param.root;
        cfg.tip         = task_param.tip;
        cfg.ref_frame   = task_param.ref_frame;
        cfg.joint_names = task_param.joint_names;
        cfg.timeout     = task_param.timeout;
        cfg.activation  = task_param.activation;
        task_config.push_back(cfg);
    }
    SceneConfig scene_cfg(params.scene.type, params.scene.file);
    scene_cfg.validate();

    RCLCPP_INFO(get_node()->get_logger(), "Configuring scene: %s", scene_cfg.type.c_str());

    PluginLoader::loadPlugin("libwbc-scenes-" + scene_cfg.type + ".so");
    scene = std::shared_ptr<Scene>(SceneFactory::createInstance(scene_cfg.type, robot_model, solver, 1.0/update_rate_));
    if(!scene->configure(task_config)){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure scene");
        return CallbackReturn::ERROR;
    }

    // Allocate memory
    joint_state.resize(robot_model->noOfJoints());
    joint_state.names = robot_model->jointNames();


    solver_output_publisher = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>("solver_output", 1);

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_activate(const rclcpp_lifecycle::State & previous_state){
    // Create state and command index maps here, since we need the interfaces to be configured first
    state_indices.clear();
    command_indices.clear();
    int idx;
    for(const std::string &joint_name : joint_state.names){
        for(const std::string &iface_name : allowed_interface_types){
            if(get_state_idx(joint_name, iface_name) != -1)
                state_indices[iface_name].push_back(get_state_idx(joint_name, iface_name));
            if(get_command_idx(joint_name, iface_name) != -1)
                command_indices[iface_name].push_back(get_command_idx(joint_name, iface_name));
        }
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    state_indices.clear();
    command_indices.clear();
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_error(const rclcpp_lifecycle::State & previous_state){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_shutdown(const rclcpp_lifecycle::State & previous_state){
    return CallbackReturn::SUCCESS;
}


/*void WholeBodyController::jointStateCallback(const sensor_msgs::msg::JointState& msg){
    fromROS(msg,joint_state);
    if(robot_model->hasFloatingBase()){
        if(has_floating_base_state)
            has_feedback = true;
    }
    else
        has_feedback = true;
}

void WholeBodyController::cartReferenceCallback(const wbc_msgs::msg::RigidBodyState& msg, const string& constraint_name){
    fromROS(msg, reference_cart);
    scene->setReference(constraint_name, reference_cart);
}

void WholeBodyController::jntReferenceCallback(const trajectory_msgs::msg::JointTrajectory& msg, const string& constraint_name){
    fromROS(msg, reference_jnt);
    scene->setReference(constraint_name, reference_jnt);
}

void WholeBodyController::taskActivationCallback(const std_msgs::msg::Float64& msg, const string& constraint_name){
    scene->setTaskActivation(constraint_name, msg.data);
}

void WholeBodyController::taskWeightsCallback(const std_msgs::msg::Float64MultiArray& msg, const string& constraint_name){
    fromROS(msg, task_weights);
    scene->setTaskWeights(constraint_name, task_weights);
}

void WholeBodyController::jointWeightsCallback(const std_msgs::msg::Float64MultiArray& msg){
    fromROS(msg, robot_model->jointNames(), joint_weights);
    scene->setJointWeights(joint_weights);
}

void WholeBodyController::floatingBaseStateCallback(const wbc_msgs::msg::RigidBodyState& msg){
    fromROS(msg, floating_base_state);
    has_floating_base_state = true;
}

void WholeBodyController::updateController(){
    timing_stats.desired_period = 1.0 / control_rate;
    timing_stats.actual_period = (get_clock()->now() - stamp).seconds();
    stamp = get_clock()->now();

    rclcpp::Time start = get_clock()->now();
    robot_model->update(joint_state, floating_base_state);
    timing_stats.time_robot_model_update = (get_clock()->now() - start).seconds();

    start = get_clock()->now();
    qp = scene->update();
    timing_stats.time_scene_update = (get_clock()->now() - start).seconds();

    start = get_clock()->now();
    solver_output = scene->solve(qp);
    timing_stats.time_solve = (get_clock()->now() - start).seconds();

    tasks_status = scene->updateTasksStatus();

    if(integrate)
        joint_integrator.integrate(robot_model->jointState(robot_model->actuatedJointNames()), solver_output, 1.0/control_rate);

    publishSolverOutput();
    publishTaskStatus();
    publishTaskInfo();

    timing_stats.time_per_cycle = (get_clock()->now() - stamp).seconds();
    timing_stats.header.stamp = get_clock()->now();
    pub_timing_stats->publish(timing_stats);
}*/
void WholeBodyController::write_command_to_hardware(){
    toROS(solver_output, solver_output_ros);
    solver_output_publisher->publish(solver_output_ros);

    // Write to hardware interfaces
    for(uint i = 0; i < solver_output.size(); i++){
        if(has_command_interface(HW_IF_POSITION))
            command_interfaces_[command_indices[HW_IF_POSITION][i]].set_value(solver_output[i].position);
        if(has_command_interface(HW_IF_VELOCITY))
            command_interfaces_[command_indices[HW_IF_VELOCITY][i]].set_value(solver_output[i].speed);
        if(has_command_interface(HW_IF_ACCELERATION))
            command_interfaces_[command_indices[HW_IF_ACCELERATION][i]].set_value(solver_output[i].acceleration);
        if(has_command_interface(HW_IF_EFFORT))
            command_interfaces_[command_indices[HW_IF_EFFORT][i]].set_value(solver_output[i].effort);
    }

    //toROS(scene->getSolverOutputRaw(), solver_output_raw);
    //solver_output_raw_publisher->publish(solver_output_raw);
}
/*
void WholeBodyController::publishTaskStatus(){
    int idx_cart = 0, idx_jnt = 0;
    for(uint i = 0; i < task_config.size(); i++){
        const TaskConfig& w = task_config[i];
        if(w.type == cart){
            toROS(robot_model->rigidBodyState(w.ref_frame, w.tip), status_cart);
            publishers_task_status_cart[idx_cart++]->publish(status_cart);
        }
        else{
            toROS(robot_model->jointState(w.joint_names), status_jnt);
            publishers_task_status_jnt[idx_jnt++]->publish(status_jnt);
        }
    }
}

void WholeBodyController::publishTaskInfo(){
    task_status_msgs.resize(tasks_status.size());
    for(uint i = 0; i < tasks_status.size(); i++){
        const TaskStatus& w = tasks_status[i];
        toROS(w, task_status_msgs[i]);
        publishers_task_info[i]->publish(task_status_msgs[i]);
    }
}*/

}

PLUGINLIB_EXPORT_CLASS(wbc_ros::WholeBodyController, controller_interface::ChainableControllerInterface)
