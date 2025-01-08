#include "wbc_ros/whole_body_controller.hpp"
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/QPSolver.hpp>
#include "conversions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <fstream>
#include <wbc/tasks/JointVelocityTask.hpp>
#include <wbc/tasks/JointAccelerationTask.hpp>
#include <wbc/tasks/CartesianVelocityTask.hpp>
#include <wbc/tasks/CartesianAccelerationTask.hpp>
#include <wbc/tasks/CoMVelocityTask.hpp>
#include <wbc/tasks/CoMAccelerationTask.hpp>
#include <wbc/tasks/WrenchForwardTask.hpp>

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

std::vector<hardware_interface::StateInterface> WholeBodyController::on_export_state_interfaces(){
    std::vector<StateInterface> interfaces;
    const string& node_name = get_node()->get_name();
    exported_state_interfaces_data.resize(14);
    uint idx = 0;
    for(const TaskPtr& task : task_config){
        switch(task->type){
            case TaskType::spatial_velocity:{
                for(auto s : pose_interfaces)
                    interfaces.push_back(StateInterface(node_name, task->config.name + "/status/" + s, exported_state_interfaces_data.data()+idx++));
                break;
            }
            default:{
                throw runtime_error("Invalid task type");
            }
        }
    }
    return interfaces;
}

std::vector<CommandInterface> WholeBodyController::on_export_reference_interfaces(){
    reference_interfaces_.resize(Scene::getNTaskVariables(task_config));
    vector<CommandInterface> interfaces;
    uint idx = 0;
    const string &node_name = get_node()->get_name();
    for(const TaskPtr& task : task_config){
       const string task_name = task->config.name;
       switch(task->type){
          case TaskType::spatial_velocity:{
              for(const string& s : twist_interfaces)
                  interfaces.push_back(CommandInterface(node_name, task_name + "/reference/" + s, reference_interfaces_.data() + idx++));
              break;
          }
          case TaskType::spatial_acceleration:{
              for(const string& s : acc_interfaces)
                  interfaces.push_back(CommandInterface(node_name, task_name + "/reference/" + s, reference_interfaces_.data() + idx++));          
              break;
          }
          case TaskType::com_velocity:{
              for(uint i = 0; i < 3; i++)
                  interfaces.push_back(CommandInterface(node_name, task_name + "/reference/" + twist_interfaces[i], reference_interfaces_.data() + idx++));
              break;
          }
          case TaskType::com_acceleration:{
              for(uint i = 0; i < 3; i++)
                  interfaces.push_back(CommandInterface(node_name, task_name + "/reference/" + acc_interfaces[i], reference_interfaces_.data() + idx++));
              break;
          }
          case TaskType::joint_velocity:{
              for(const string& name : dynamic_pointer_cast<JointVelocityTask>(task)->jointNames())
                  interfaces.push_back(CommandInterface(node_name, task_name + "/reference/" + name + "/velocity", reference_interfaces_.data() + idx++));          
              break;
          }
          case TaskType::joint_acceleration:{
              for(const string& name : dynamic_pointer_cast<JointAccelerationTask>(task)->jointNames())
                  interfaces.push_back(CommandInterface(node_name, task_name + "/reference/" + name + "/acceleration", reference_interfaces_.data() + idx++));          
              break;
          }
          case TaskType::wrench_forward:{
              for(const string& s : wrench_interfaces)
                  interfaces.push_back(CommandInterface(node_name, task_name + "/reference/" + s, reference_interfaces_.data() + idx++));          
              break;
          }
          default:{
              throw runtime_error("Invalid task type");
          }                                                                      
       }
    }
    return interfaces;
}

controller_interface::return_type WholeBodyController::update_reference_from_subscribers(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    return controller_interface::return_type::OK;
}

void WholeBodyController::read_state_from_hardware(){
    for(uint i = 0; i < robot_model->nj(); i++){
        if(has_state_interface(HW_IF_POSITION))
            joint_state.position[i] = state_interfaces_[state_indices[HW_IF_POSITION][i]].get_value();
        if(has_state_interface(HW_IF_VELOCITY))
            joint_state.velocity[i] = state_interfaces_[state_indices[HW_IF_VELOCITY][i]].get_value();
        if(has_state_interface(HW_IF_ACCELERATION))
            joint_state.acceleration[i] = state_interfaces_[state_indices[HW_IF_ACCELERATION][i]].get_value();
    }
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
    robot_model_cfg.submechanism_file    = params.robot_model.submechanism_file;
    robot_model_cfg.floating_base        = params.robot_model.floating_base;
    for(auto name : params.contact_names){
        const auto &c = params.robot_model.contact_points.contact_names_map.at(name);
        robot_model_cfg.contact_points.push_back(Contact(name, c.active, c.mu, c.wx, c.wy));
    }

    RCLCPP_INFO(get_node()->get_logger(), "Configuring robot model: %s", params.robot_model.type.c_str());
    PluginLoader::loadPlugin("libwbc-robot_models-" + params.robot_model.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(params.robot_model.type));
    if(!robot_model->configure(robot_model_cfg)){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure robot model");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Configuring solver: %s", params.solver.type.c_str());
    PluginLoader::loadPlugin("libwbc-solvers-" + params.solver.type + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(params.solver.type));

    task_config.clear();
    for(auto task_name : params.task_names){
        TaskPtr task;
        const auto& task_param = params.tasks.task_names_map.at(task_name);
        switch(fromString(task_param.type)){
            case TaskType::spatial_velocity:{
                task = make_shared<CartesianVelocityTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                          robot_model,
                                                          task_param.tip_frame,
                                                          task_param.ref_frame);
                break;
            }
            case TaskType::spatial_acceleration:{
                task = make_shared<CartesianAccelerationTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                              robot_model,
                                                              task_param.tip_frame,
                                                              task_param.ref_frame);
                break;
            }
            case TaskType::com_velocity:{
                task = make_shared<CoMVelocityTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                    robot_model);
                break;
            }
            case TaskType::com_acceleration:{
                task = make_shared<CoMAccelerationTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                        robot_model);
                break;
            }
            case TaskType::joint_velocity:{
                task = make_shared<JointVelocityTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                      robot_model,
                                                      task_param.joint_names);
                break;
            }
            case TaskType::joint_acceleration:{
                task = make_shared<JointAccelerationTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                          robot_model,
                                                          task_param.joint_names);
                break;
            }
            case TaskType::wrench_forward:{
                task = make_shared<WrenchForwardTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                      robot_model,
                                                      task_param.ref_frame);
                break;
            }
            default:{
                RCLCPP_ERROR(get_node()->get_logger(), string("Invalid task type: " + task_param.type).c_str());
                return CallbackReturn::ERROR;
            }
        }
        task_config.push_back(task);
    }

    RCLCPP_INFO(get_node()->get_logger(), "Configuring scene: %s", params.scene.type.c_str());

    PluginLoader::loadPlugin("libwbc-scenes-" + params.scene.type + ".so");
    scene = std::shared_ptr<Scene>(SceneFactory::createInstance(params.scene.type, robot_model, solver, 0.001));
    if(!scene->configure(task_config)){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure scene");
        return CallbackReturn::ERROR;
    }

    // Allocate memory
    joint_state.resize(robot_model->nj());
    solver_output.resize(robot_model->na());

    // Subscribers/Publishers

    // Solver output for easier debugging
    solver_output_publisher = get_node()->create_publisher<CommandMsg>("~/solver_output", rclcpp::SystemDefaultsQoS());
    rt_solver_output_publisher = make_unique<RTCommandPublisher>(solver_output_publisher);

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
    robot_model->update(joint_state.position,
                        joint_state.velocity,
                        joint_state.acceleration,
                        floating_base_state.pose,
                        floating_base_state.twist,
                        floating_base_state.acceleration);
    timing_stats.time_robot_model_update = (get_node()->get_clock()->now() - start).seconds();

    start = get_node()->get_clock()->now();

    update_tasks();

    qp = scene->update();
    timing_stats.time_scene_update = (get_node()->get_clock()->now() - start).seconds();

    start = get_node()->get_clock()->now();
    solver_output = scene->solve(qp);
    timing_stats.time_solve = (get_node()->get_clock()->now() - start).seconds();

    if(params.control_mode == "velocity")
        joint_integrator.integrate(robot_model->jointState(), solver_output, 1.0/update_rate, types::CommandMode::VELOCITY);
    else
        joint_integrator.integrate(robot_model->jointState(), solver_output, 1.0/update_rate, types::CommandMode::ACCELERATION);
    
    write_command_to_hardware();
    write_to_state_interfaces();

    timing_stats.time_per_cycle = (get_node()->get_clock()->now() - stamp).seconds();
    timing_stats.header.stamp = get_node()->get_clock()->now();
    rt_timing_stats_publisher->lock();
    rt_timing_stats_publisher->msg_ = timing_stats;
    rt_timing_stats_publisher->unlockAndPublish();
    return controller_interface::return_type::OK;
}

void WholeBodyController::write_to_state_interfaces(){
    uint idx = 0;
    for(uint i = 0; i < task_config.size(); i++){
        TaskPtr task = task_config[i];
        switch(task->type){
            case spatial_velocity:{
                const string& tip = dynamic_pointer_cast<CartesianVelocityTask>(task)->tipFrame();
                types::Pose pose = robot_model->pose(tip);
                for(uint i = 0; i < 3; i++)
                    ordered_exported_state_interfaces_[idx++]->set_value(pose.position[i]);
                for(uint i = 0; i < 4; i++)
                    ordered_exported_state_interfaces_[idx++]->set_value(pose.orientation.coeffs()[i]);
                break;
            }
            case com_velocity:
            case com_acceleration:{
                // TODO: to state interfaces
                break;
            }
            case joint_velocity:
            case joint_acceleration:{
                // No extra state interfacesneeded for joint space tasks
                break;
            }
            default:{
                throw std::runtime_error("Invalid task type");
            }
        }
    }
}

void WholeBodyController::update_tasks(){
    uint idx = 0;
    for(TaskPtr& task : task_config){
        switch(task->type){
            case TaskType::spatial_velocity:{
                reference_cart.twist.linear  = Eigen::Vector3d(reference_interfaces_[idx],  reference_interfaces_[idx+1],reference_interfaces_[idx+2]);
                reference_cart.twist.angular = Eigen::Vector3d(reference_interfaces_[idx+3],reference_interfaces_[idx+4],reference_interfaces_[idx+5]);
                idx+=6;
                dynamic_pointer_cast<CartesianVelocityTask>(task)->setReference(reference_cart.twist);
                break;
            }
            case TaskType::spatial_acceleration:{
                reference_cart.acceleration.linear  = Eigen::Vector3d(reference_interfaces_[idx],  reference_interfaces_[idx+1],reference_interfaces_[idx+2]);
                reference_cart.acceleration.angular = Eigen::Vector3d(reference_interfaces_[idx+3],reference_interfaces_[idx+4],reference_interfaces_[idx+5]);
                idx+=6;
                dynamic_pointer_cast<CartesianAccelerationTask>(task)->setReference(reference_cart.acceleration);
                break;
            }
            case TaskType::com_velocity:{
                reference_cart.twist.linear  = Eigen::Vector3d(reference_interfaces_[idx],  reference_interfaces_[idx+1],reference_interfaces_[idx+2]);
                reference_cart.twist.angular = Eigen::Vector3d(reference_interfaces_[idx+3],reference_interfaces_[idx+4],reference_interfaces_[idx+5]);
                idx+=6;
                dynamic_pointer_cast<CoMVelocityTask>(task)->setReference(reference_cart.twist.linear);
                break;
            }
            case TaskType::com_acceleration:{
                reference_cart.acceleration.linear  = Eigen::Vector3d(reference_interfaces_[idx],  reference_interfaces_[idx+1],reference_interfaces_[idx+2]);
                reference_cart.acceleration.angular = Eigen::Vector3d(reference_interfaces_[idx+3],reference_interfaces_[idx+4],reference_interfaces_[idx+5]);
                idx+=6;
                dynamic_pointer_cast<CoMAccelerationTask>(task)->setReference(reference_cart.acceleration.linear);            
                break;
            }
            case TaskType::joint_velocity:{
                reference_jnt.resize(task->nv);
                for(uint i = 0; i < task->nv; i++)
                    reference_jnt.velocity[i] = reference_interfaces_[idx++];
                dynamic_pointer_cast<JointVelocityTask>(task)->setReference(reference_jnt.velocity);            
                break;
            }
            case TaskType::joint_acceleration:{
                reference_jnt.resize(task->nv);
                for(uint i = 0; i < task->nv; i++)
                    reference_jnt.acceleration[i] = reference_interfaces_[idx++];
                dynamic_pointer_cast<JointAccelerationTask>(task)->setReference(reference_jnt.acceleration);              
                break;
            }
            case TaskType::wrench_forward:{
                reference_wrench.force  = Eigen::Vector3d(reference_interfaces_[idx],  reference_interfaces_[idx+1],reference_interfaces_[idx+2]);
                reference_wrench.torque = Eigen::Vector3d(reference_interfaces_[idx+3],reference_interfaces_[idx+4],reference_interfaces_[idx+5]);
                idx+=6;
                dynamic_pointer_cast<WrenchForwardTask>(task)->setReference(reference_wrench);            
                break;
            }
            default:{
                assert("Invalid task type");
            }
        }
    }

    // Update task weights
    task_weight_msg = *rt_task_weight_buffer.readFromRT();
    if(task_weight_msg.get()){
        TaskPtr task = scene->getTask(task_weight_msg->task_name);
        task->setWeights(Eigen::Map<Eigen::VectorXd>(task_weight_msg->weights.data(), task_weight_msg->weights.size()));
    }

    // Update task activations
    task_activation_msg = *rt_task_activation_buffer.readFromRT();
    if (task_activation_msg.get()){
        TaskPtr task = scene->getTask(task_activation_msg->task_name);
        task->setActivation(task_activation_msg->activation);
    }
}

void WholeBodyController::write_command_to_hardware(){
    //rt_solver_output_publisher->lock();    
    //toROS(solver_output, robot_model->jointNames(), rt_solver_output_publisher->msg_);
    //rt_solver_output_publisher->unlockAndPublish();

    // Write to hardware interfaces
    for(uint i = 0; i < robot_model->na(); i++){
        if(has_command_interface(HW_IF_POSITION))
            command_interfaces_[command_indices[HW_IF_POSITION][i]].set_value(solver_output.position[i]);
        if(has_command_interface(HW_IF_VELOCITY))
            command_interfaces_[command_indices[HW_IF_VELOCITY][i]].set_value(solver_output.velocity[i]);
        if(has_command_interface(HW_IF_ACCELERATION))
            command_interfaces_[command_indices[HW_IF_ACCELERATION][i]].set_value(solver_output.acceleration[i]);
        if(has_command_interface(HW_IF_EFFORT))
            command_interfaces_[command_indices[HW_IF_EFFORT][i]].set_value(solver_output.effort[i]);
    }
}

controller_interface::CallbackReturn WholeBodyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    // Create state and command index maps here, since we need the interfaces to be configured first
    state_indices.clear();
    command_indices.clear();
    for(const std::string &joint_name : robot_model->jointNames()){
        for(const std::string &iface_name : allowed_interface_types){
            if(get_state_idx(joint_name, iface_name) != -1)
                state_indices[iface_name].push_back(get_state_idx(joint_name, iface_name));
            if(get_command_idx(joint_name, iface_name) != -1)
                command_indices[iface_name].push_back(get_command_idx(joint_name, iface_name));
        }
    }

    //Clear all task references, weights etc. to have to secure initial state
    for(const auto &task : task_config)
       scene->getTask(task->config.name)->reset();

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
