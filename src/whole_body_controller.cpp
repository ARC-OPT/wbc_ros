#include <wbc_ros/whole_body_controller.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/tasks/JointVelocityTask.hpp>
#include <wbc/tasks/JointAccelerationTask.hpp>
#include <wbc/tasks/SpatialVelocityTask.hpp>
#include <wbc/tasks/SpatialAccelerationTask.hpp>
#include <wbc/tasks/CoMVelocityTask.hpp>
#include <wbc/tasks/CoMAccelerationTask.hpp>
#include <wbc/tasks/ContactForceTask.hpp>
#include <wbc/controllers/CartesianPosPDController.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std;
using namespace wbc;

namespace wbc_ros{

WholeBodyController::WholeBodyController(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("whole_body_controller", options){
    // Create the parameter listener and read all ROS parameters
    param_listener = std::make_shared<whole_body_controller::ParamListener>(this->get_node_parameters_interface());
    params = param_listener->get_params();
}

void WholeBodyController::joint_weight_callback(const JointWeightMsgPtr msg){
    rt_joint_weight_buffer.writeFromNonRT(msg);
}

void WholeBodyController::robot_state_callback(const RobotStateMsgPtr msg){
    has_robot_state = true;
    rt_robot_state_buffer.writeFromNonRT(msg);
}

void WholeBodyController::contacts_callback(const ContactsMsgPtr msg){
    rt_contacts_buffer.writeFromNonRT(msg);
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn WholeBodyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    std::string urdf_string;
    while (!this->get_parameter("robot_description", urdf_string)){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for parameter %s.", "/robot_description");
        usleep(100000);
    }
    while (!this->get_parameter("update_rate", update_rate)){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for parameter %s.", "/update_rate");
        usleep(100000);
    }
    if(update_rate <= 0){
        RCLCPP_FATAL(this->get_logger(), "Update Rate parameter is %i. Did you forget to set it?", update_rate);
        throw std::runtime_error("Invalid update_rate parameter");
    }

    RobotModelConfig robot_model_cfg;
    robot_model_cfg.file_or_string       = urdf_string;
    robot_model_cfg.joint_blacklist      = params.robot_model.joint_blacklist;
    robot_model_cfg.submechanism_file    = params.robot_model.submechanism_file;
    robot_model_cfg.floating_base        = params.robot_model.floating_base;
    for(auto name : params.contact_names){
        const auto &c = params.robot_model.contact_points.contact_names_map.at(name);
        robot_model_cfg.contact_points.push_back(types::Contact(name, c.active, c.mu, c.wx, c.wy));
    }       

    RCLCPP_INFO(this->get_logger(), "Configuring robot model: %s", params.robot_model.type.c_str());
    PluginLoader::loadPlugin("libwbc-robot_models-" + params.robot_model.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(params.robot_model.type));
    if(!robot_model->configure(robot_model_cfg)){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure robot model");
        return CallbackReturn::ERROR;
    }
    //contacts_interface = make_shared<ContactsInterface>(robot_model);

    RCLCPP_INFO(this->get_logger(), "Configuring solver: %s", params.solver.type.c_str());
    PluginLoader::loadPlugin("libwbc-solvers-" + params.solver.type + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(params.solver.type));

    vector<TaskPtr> tasks;
    for(auto task_name : params.task_names){
        TaskPtr task;
        TaskInterfacePtr iface;
        auto task_param = params.tasks.task_names_map.at(task_name);
        auto ctrl_param = params.controllers.task_names_map.at(task_name);
        switch(task_param.type){
            case TaskType::spatial_velocity:{
                task = std::make_shared<SpatialVelocityTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                             robot_model, 
                                                             task_param.tip_frame, 
                                                             task_param.ref_frame);
                CartesianPosPDControllerPtr controller = std::make_shared<CartesianPosPDController>();    
                controller->setPGain(Eigen::Map<Eigen::VectorXd>(ctrl_param.p_gain.data(), ctrl_param.p_gain.size()));
                controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(ctrl_param.max_control_output.data(), ctrl_param.max_control_output.size()));                                                         
                iface = std::make_shared<SpatialVelocityTaskInterface>(task, controller, robot_model, this->shared_from_this());                
                break;
            }
            case TaskType::spatial_acceleration:{
                task = std::make_shared<SpatialAccelerationTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                                 robot_model,
                                                                 task_param.tip_frame,
                                                                 task_param.ref_frame);
                CartesianPosPDControllerPtr controller = std::make_shared<CartesianPosPDController>();
                controller->setPGain(Eigen::Map<Eigen::VectorXd>(ctrl_param.p_gain.data(), ctrl_param.p_gain.size()));
                controller->setDGain(Eigen::Map<Eigen::VectorXd>(ctrl_param.d_gain.data(), ctrl_param.d_gain.size()));
                controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(ctrl_param.max_control_output.data(), ctrl_param.max_control_output.size()));                                                     
                iface = std::make_shared<SpatialAccelerationTaskInterface>(task, controller, robot_model, this->shared_from_this());
                break;
            }
            case TaskType::com_velocity:{
                task = std::make_shared<CoMVelocityTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                         robot_model);
                break;
            }
            case TaskType::com_acceleration:{
                task = std::make_shared<CoMAccelerationTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                        robot_model);                                                       
                break;
            }
            case TaskType::joint_velocity:{
                task = std::make_shared<JointVelocityTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                           robot_model,
                                                           robot_model->jointNames());
                JointPosPDControllerPtr controller = std::make_shared<JointPosPDController>(robot_model->nj());
                controller->setPGain(Eigen::Map<Eigen::VectorXd>(ctrl_param.p_gain.data(), ctrl_param.p_gain.size()));
                controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(ctrl_param.max_control_output.data(), ctrl_param.max_control_output.size()));  
                iface = std::make_shared<JointVelocityTaskInterface>(task, controller, robot_model, this->shared_from_this());
                break;
            }
            case TaskType::joint_acceleration:{
                task = std::make_shared<JointAccelerationTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                               robot_model,
                                                               robot_model->jointNames());
                JointPosPDControllerPtr controller = std::make_shared<JointPosPDController>(robot_model->nj());
                controller->setPGain(Eigen::Map<Eigen::VectorXd>(ctrl_param.p_gain.data(), ctrl_param.p_gain.size()));
                controller->setDGain(Eigen::Map<Eigen::VectorXd>(ctrl_param.d_gain.data(), ctrl_param.d_gain.size()));
                controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(ctrl_param.max_control_output.data(), ctrl_param.max_control_output.size()));  
                iface = std::make_shared<JointAccelerationTaskInterface>(task, controller, robot_model, this->shared_from_this());  
                break;
            }
            case TaskType::contact_force:{
                task = std::make_shared<ContactForceTask>(TaskConfig(task_name, task_param.priority, task_param.weights, task_param.activation),
                                                          robot_model,
                                                          task_param.ref_frame);
                break;
            }
            default:{
                RCLCPP_ERROR(this->get_logger(), string("Invalid task type: " + task_param.type).c_str());
                return CallbackReturn::ERROR;
            }
        }
        task_interfaces.push_back(iface);
        tasks.push_back(task);
    }

    RCLCPP_INFO(this->get_logger(), "Configuring scene: %s", params.scene.type.c_str());

    PluginLoader::loadPlugin("libwbc-scenes-" + params.scene.type + ".so");
    scene = std::shared_ptr<Scene>(SceneFactory::createInstance(params.scene.type, robot_model, solver, 0.001));
    if(!scene->configure(tasks)){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure scene");
        return CallbackReturn::ERROR;
    }

    // Allocate memory
    joint_state.resize(robot_model->nj());
    solver_output.resize(robot_model->na());

    // Subscribers/Publishers

    // Solver output for easier debugging
    solver_output_publisher = this->create_publisher<CommandMsg>("~/solver_output", rclcpp::SystemDefaultsQoS());
    rt_solver_output_publisher = std::make_unique<RTCommandPublisher>(solver_output_publisher);

    robot_state_subscriber = this->create_subscription<RobotStateMsg>("~/robot_state",
        rclcpp::SystemDefaultsQoS(), std::bind(&WholeBodyController::robot_state_callback, this, placeholders::_1));

    timing_stats_publisher = this->create_publisher<TimingStatsMsg>("~/timing_stats", rclcpp::SystemDefaultsQoS());
    rt_timing_stats_publisher = std::make_unique<RTTimingStatsPublisher>(timing_stats_publisher);

    joint_weight_subscriber = this->create_subscription<JointWeightMsg>("~/joint_weights", 
        rclcpp::SystemDefaultsQoS(), std::bind(&WholeBodyController::joint_weight_callback, this, std::placeholders::_1));

    timer = this->create_wall_timer(std::chrono::milliseconds((int)1000.0/params.update_rate),std::bind(&WholeBodyController::updateController, this));
    timer->cancel(); 

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void WholeBodyController::updateController(){

    timing_stats.desired_period = 1.0/update_rate;
    if(stamp.nanoseconds() != 0)
        timing_stats.actual_period = (this->get_clock()->now() - stamp).seconds();
    stamp = this->get_clock()->now();

    if(!has_robot_state){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No robot state");
        return;
    }

    // 1. Update the internal robot model with the current robot state (joint state + floating base + contacts)
    rclcpp::Time start = this->get_clock()->now();
    robot_state_msg = *rt_robot_state_buffer.readFromRT();   
    if(robot_state_msg->joint_state.position.size() > 0)
        fromROS(*robot_state_msg, joint_state, floating_base_state);
    else
        return;
        
    robot_model->update(joint_state.position, joint_state.velocity, joint_state.acceleration,
                        floating_base_state.pose, floating_base_state.twist, floating_base_state.acceleration);
    timing_stats.time_robot_model_update = (this->get_clock()->now() - start).seconds();

    contacts_msg = *rt_contacts_buffer.readFromRT();
    if(contacts_msg.get()){
        fromROS(*contacts_msg, contacts);
        robot_model->setContacts(contacts);
    }

    start = this->get_clock()->now();

    // 2. Update Tasks (Task weights, joint weights, Controllers)
    for(auto task : task_interfaces){
        task->updateTaskWeights();
        task->updateTask();
    }
    joint_weight_msg = *rt_joint_weight_buffer.readFromRT();
    if(joint_weight_msg.get())
        robot_model->setJointWeights(Eigen::Map<Eigen::VectorXd>(joint_weight_msg->data.data(), joint_weight_msg->data.size())); 

    // 3. Set up QP
    qp = scene->update();
    timing_stats.time_scene_update = (this->get_clock()->now() - start).seconds();

    // 4. Solve the QP 
    start = this->get_clock()->now();
    solver_output = scene->solve(qp);
    timing_stats.time_solve = (this->get_clock()->now() - start).seconds();

    // 5. Integrate and write the solution to topic
    if(params.control_mode == ControlMode::velocity)
        joint_integrator.integrate(robot_model->jointState(), solver_output, 1.0/update_rate, types::CommandMode::VELOCITY);
    else
        joint_integrator.integrate(robot_model->jointState(), solver_output, 1.0/update_rate, types::CommandMode::ACCELERATION);    

    timing_stats.time_per_cycle = (this->get_clock()->now() - stamp).seconds();
    rt_timing_stats_publisher->lock();
    rt_timing_stats_publisher->msg_ = timing_stats;
    rt_timing_stats_publisher->unlockAndPublish();

    rt_solver_output_publisher->lock();    
    toROS(solver_output, rt_solver_output_publisher->msg_);
    rt_solver_output_publisher->unlockAndPublish();

    timing_stats.time_per_cycle = (this->get_clock()->now() - stamp).seconds();
    rt_timing_stats_publisher->lock();
    rt_timing_stats_publisher->msg_ = timing_stats;
    rt_timing_stats_publisher->unlockAndPublish();
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn WholeBodyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    has_robot_state = false;
    
    // Clear all task references, weights etc. to have to secure initial state
    for(auto ti : task_interfaces)
        ti->task->reset();

    // Also reinit the integrator as it will store the last joint position as a starting point
    joint_integrator.reinit();
    timer->reset(); 
    
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn WholeBodyController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    timer->reset(); 
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn WholeBodyController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    joint_state.clear();
    solver_output.clear();
    joint_integrator.reinit();

    PluginLoader::unloadPlugin("libwbc-robot_models-" + params.robot_model.type + ".so");
    PluginLoader::unloadPlugin("libwbc-solvers-" + params.solver.type + ".so");
    PluginLoader::unloadPlugin("libwbc-scenes-" + params.scene.type + ".so");

    RobotModelFactory::clear();
    QPSolverFactory::clear();
    SceneFactory::clear();

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn WholeBodyController::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn WholeBodyController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(wbc_ros::WholeBodyController)