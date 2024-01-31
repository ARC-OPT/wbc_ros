#include "wbc_ros/whole_body_controller.hpp"
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/QPSolver.hpp>
#include "conversions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <fstream>

using namespace std;
using namespace wbc;
using namespace hardware_interface;

namespace wbc_ros{

WholeBodyController::WholeBodyController() : ChainableControllerInterface(), has_floating_base_state(false){
}

void WholeBodyController::task_weight_callback(const TaskWeightMsgPtr msg){
    rt_task_weight_buffer.writeFromNonRT(msg);
}

void WholeBodyController::task_activation_callback(const TaskActivationMsgPtr msg){
    rt_task_activation_buffer.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration WholeBodyController::command_interface_configuration() const{
    vector<string> iface_names;
    for(const string &joint_name : robot_model->jointNames()){
        for(const string &iface_name : params.command_interfaces)
            iface_names.push_back(params.prefix + joint_name + "/" + iface_name);
    }
    return { controller_interface::interface_configuration_type::INDIVIDUAL, iface_names};
}

controller_interface::InterfaceConfiguration WholeBodyController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    if(params.state_interfaces.empty())
        conf.type = controller_interface::interface_configuration_type::ALL;
    else{
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        vector<string> iface_names;
        for(const string &joint_name : robot_model->jointNames()){
            for(const string &iface_name : params.state_interfaces)
                iface_names.push_back(params.prefix + joint_name + "/" + iface_name);
        }
        conf.names = iface_names;
    }
    return conf;
}

std::vector<CommandInterface> WholeBodyController::on_export_reference_interfaces(){
    reference_interfaces_.resize(Scene::getNTaskVariables(task_config));
    vector<CommandInterface> interfaces;
    uint idx = 0;
    for(const TaskConfig& w : task_config){
        if(w.type == cart || w.type == com){
            if(params.control_mode == "velocity"){
                for(const string& s : cart_vel_interfaces)
                    interfaces.push_back(CommandInterface(string(get_node()->get_name()), w.name + "/reference/" + s, reference_interfaces_.data() + idx++));
            }
            else if(params.control_mode == "acceleration"){
                for(const string& s : cart_acc_interfaces)
                    interfaces.push_back(CommandInterface(string(get_node()->get_name()), w.name + "/reference/" + s, reference_interfaces_.data() + idx++));
            }
        }
        else{ // w.type == jnt
            if(params.control_mode == "velocity"){
                for(const string& name : w.joint_names)
                    interfaces.push_back(CommandInterface(string(get_node()->get_name()), w.name + "/reference/" + name + "/velocity", reference_interfaces_.data() + idx++));
            }
            if(params.control_mode == "acceleration"){
                for(const string& name : w.joint_names)
                    interfaces.push_back(CommandInterface(string(get_node()->get_name()), w.name + "/reference/" + name + "/acceleration", reference_interfaces_.data() + idx++));
            }
        }
    }
    return interfaces;
}

controller_interface::return_type WholeBodyController::update_reference_from_subscribers(){
    return controller_interface::return_type::OK;
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

controller_interface::CallbackReturn WholeBodyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

    std::string urdf_string;
    while (!get_node()->get_parameter("robot_description", urdf_string)){
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "Waiting for parameter %s.", "/robot_description");
        usleep(100000);
    }
    while (!get_node()->get_parameter("update_rate", update_rate)){
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "Waiting for parameter %s.", "/update_rate");
        usleep(100000);
    }
    if(update_rate <= 0){
        RCLCPP_FATAL(get_node()->get_logger(), "Update Rate parameter is %i. Did you forget to set it?", update_rate);
        throw std::runtime_error("Invalid update_rate parameter");
    }

    RobotModelConfig robot_model_cfg;
    robot_model_cfg.file_or_string       = urdf_string;
    robot_model_cfg.joint_blacklist      = params.robot_model.joint_blacklist;
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
    scene = std::shared_ptr<Scene>(SceneFactory::createInstance(scene_cfg.type, robot_model, solver, 0.001));
    if(!scene->configure(task_config)){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure scene");
        return CallbackReturn::ERROR;
    }

    // Allocate memory
    joint_state.resize(robot_model->noOfJoints());
    joint_state.names = robot_model->jointNames();

    solver_output.resize(robot_model->noOfActuatedJoints());
    solver_output.names = robot_model->actuatedJointNames();

    // Subscribers/Publishers

    // Solver output for easier debugging
    solver_output_publisher = get_node()->create_publisher<CommandMsg>("~/solver_output", rclcpp::SystemDefaultsQoS());
    rt_solver_output_publisher = make_unique<RTCommandPublisher>(solver_output_publisher);

    // Task status as feedback for controllers
    for(auto w : task_config){
        if(w.type == cart || w.type == com){
            RbsPublisher::SharedPtr pub = get_node()->create_publisher<RbsMsg>("~/status_" + w.name, rclcpp::SystemDefaultsQoS());
            task_status_publishers_cart.push_back(pub);
            rt_task_status_publishers_cart.push_back(make_unique<RTRbsPublisher>(pub));
        }
        else{
            JointsPublisher::SharedPtr pub = get_node()->create_publisher<JointsMsg>("~/status_" + w.name, rclcpp::SystemDefaultsQoS());
            task_status_publishers_jnt.push_back(pub);
            rt_task_status_publishers_jnt.push_back(make_unique<RTJointsPublisher>(pub));
        }
    }

    timing_stats_publisher = get_node()->create_publisher<TimingStatsMsg>("~/timing_stats", rclcpp::SystemDefaultsQoS());
    rt_timing_stats_publisher = make_unique<RTTimingStatsPublisher>(timing_stats_publisher);

    task_weight_subscriber = get_node()->create_subscription<TaskWeightMsg>("~/task_weights",
        rclcpp::SystemDefaultsQoS(), bind(&WholeBodyController::task_weight_callback, this, placeholders::_1));

    task_activation_subscriber = get_node()->create_subscription<TaskActivationMsg>("~/task_activation",
        rclcpp::SystemDefaultsQoS(), bind(&WholeBodyController::task_activation_callback, this, placeholders::_1));

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type WholeBodyController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    timing_stats.desired_period = 1.0/update_rate;
    if(stamp.nanoseconds() != 0)
        timing_stats.actual_period = (get_node()->get_clock()->now() - stamp).seconds();
    stamp = get_node()->get_clock()->now();

    read_state_from_hardware();

    rclcpp::Time start = get_node()->get_clock()->now();
    robot_model->update(joint_state, floating_base_state);
    timing_stats.time_robot_model_update = (get_node()->get_clock()->now() - start).seconds();

    start = get_node()->get_clock()->now();

    update_tasks();

    qp = scene->update();
    timing_stats.time_scene_update = (get_node()->get_clock()->now() - start).seconds();

    start = get_node()->get_clock()->now();
    solver_output = scene->solve(qp);
    timing_stats.time_solve = (get_node()->get_clock()->now() - start).seconds();

    //tasks_status = scene->updateTasksStatus();

    joint_integrator.integrate(robot_model->jointState(robot_model->actuatedJointNames()), solver_output, 1.0/update_rate);

    write_command_to_hardware();
    publish_task_status();
    // publishTaskInfo();

    timing_stats.time_per_cycle = (get_node()->get_clock()->now() - stamp).seconds();
    timing_stats.header.stamp = get_node()->get_clock()->now();
    rt_timing_stats_publisher->lock();
    rt_timing_stats_publisher->msg_ = timing_stats;
    rt_timing_stats_publisher->unlockAndPublish();
    return controller_interface::return_type::OK;
}

void WholeBodyController::publish_task_status(){
    int idx_cart = 0, idx_jnt = 0;
    for(uint i = 0; i < task_config.size(); i++){
        const TaskConfig& w = task_config[i];
        if(w.type == cart){
            rt_task_status_publishers_cart[idx_cart]->lock();
            toROS(robot_model->rigidBodyState(w.ref_frame, w.tip), rt_task_status_publishers_cart[idx_cart]->msg_);
            rt_task_status_publishers_cart[idx_cart]->unlockAndPublish();
            idx_cart++;
        }
        else if(w.type == com){
            rt_task_status_publishers_cart[idx_cart]->lock();
            toROS(robot_model->centerOfMass(), rt_task_status_publishers_cart[idx_cart]->msg_);
            rt_task_status_publishers_cart[idx_cart]->unlockAndPublish();
            idx_cart++;
        }
        else{
            rt_task_status_publishers_jnt[idx_jnt]->lock();
            toROS(robot_model->jointState(w.joint_names), rt_task_status_publishers_jnt[idx_jnt]->msg_);
            rt_task_status_publishers_jnt[idx_jnt]->unlockAndPublish();
            idx_jnt++;
        }
    }
}

void WholeBodyController::update_tasks(){
    uint idx = 0;
    for(const TaskConfig& w : task_config){
        if(w.type == cart || w.type == com){
            if(params.control_mode == "velocity"){
                reference_cart.twist.linear  = base::Vector3d(reference_interfaces_[idx],  reference_interfaces_[idx+1],reference_interfaces_[idx+2]);
                reference_cart.twist.angular = base::Vector3d(reference_interfaces_[idx+3],reference_interfaces_[idx+4],reference_interfaces_[idx+5]);
            }
            else{
                reference_cart.acceleration.linear  = base::Vector3d(reference_interfaces_[idx],  reference_interfaces_[idx+1],reference_interfaces_[idx+2]);
                reference_cart.acceleration.angular = base::Vector3d(reference_interfaces_[idx+3],reference_interfaces_[idx+4],reference_interfaces_[idx+5]);
            }
            idx+=6;
            scene->setReference(w.name, reference_cart);
        }
        else{
            reference_jnt.resize(w.joint_names.size());
            reference_jnt.names = w.joint_names;
            if(params.control_mode == "velocity"){
                for(const string& name : w.joint_names)
                    reference_jnt[name].speed = reference_interfaces_[idx++];
            }
            else{
                for(const string& name : w.joint_names)
                    reference_jnt[name].acceleration = reference_interfaces_[idx++];
            }
            scene->setReference(w.name, reference_jnt);
        }
    }

    // Update task weights
    task_weight_msg = *rt_task_weight_buffer.readFromRT();
    if (task_weight_msg.get())
        scene->setTaskWeights(task_weight_msg->task_name, Eigen::Map<Eigen::VectorXd>(task_weight_msg->weights.data(), task_weight_msg->weights.size()));

    // Update task activations
    task_activation_msg = *rt_task_activation_buffer.readFromRT();
    if (task_activation_msg.get())
        scene->setTaskActivation(task_activation_msg->task_name, task_activation_msg->activation);
}

void WholeBodyController::write_command_to_hardware(){
    rt_solver_output_publisher->lock();
    toROS(solver_output, rt_solver_output_publisher->msg_);
    rt_solver_output_publisher->unlockAndPublish();

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
}

controller_interface::CallbackReturn WholeBodyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    // Create state and command index maps here, since we need the interfaces to be configured first
    state_indices.clear();
    command_indices.clear();
    for(const std::string &joint_name : joint_state.names){
        for(const std::string &iface_name : allowed_interface_types){
            if(get_state_idx(joint_name, iface_name) != -1)
                state_indices[iface_name].push_back(get_state_idx(joint_name, iface_name));
            if(get_command_idx(joint_name, iface_name) != -1)
                command_indices[iface_name].push_back(get_command_idx(joint_name, iface_name));
        }
    }

    //Clear all task references, weights etc. to have to secure initial state
    for(const auto &cfg : task_config)
       scene->getTask(cfg.name)->reset();

    // Also reinit the integrate as it will store the last joint position as a starting point
    joint_integrator.reinit();
    
    // Reset the reference interfaces (outputs from controllers) to not store old data
    for(uint i = 0; i < reference_interfaces_.size(); i++)
        reference_interfaces_[i] = 0.0;
    
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    state_indices.clear();
    command_indices.clear();
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    joint_state.clear();
    solver_output.clear();
    joint_integrator.reinit();

    PluginLoader::unloadPlugin("libwbc-robot_models-" + params.robot_model.type + ".so");
    PluginLoader::unloadPlugin("libwbc-solvers-" + params.solver.type + ".so");
    PluginLoader::unloadPlugin("libwbc-scenes-" + params.scene.type + ".so");

    RobotModelFactory::clear();
    QPSolverFactory::clear();
    SceneFactory::clear();

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;
}
}

PLUGINLIB_EXPORT_CLASS(wbc_ros::WholeBodyController, controller_interface::ChainableControllerInterface)
