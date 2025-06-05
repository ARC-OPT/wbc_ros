#include <wbc_ros/biped_controller.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <wbc/core/PluginLoader.hpp>

using namespace std;
using namespace wbc;
using namespace wbc::types;

namespace wbc_ros{

BipedController::BipedController(const rclcpp::NodeOptions & options) : 
    rclcpp_lifecycle::LifecycleNode("whole_body_controller", options){
    param_listener = std::make_shared<biped_controller::ParamListener>(this->get_node_parameters_interface());
    params = param_listener->get_params();
}

void BipedController::joint_weight_callback(const DoubleArrayMsgPtr msg){
    rt_joint_weight_buffer.writeFromNonRT(msg);
}

void BipedController::robot_state_callback(const RobotStateMsgPtr msg){
    has_robot_state = true;
    rt_robot_state_buffer.writeFromNonRT(msg);
}

void BipedController::joint_state_callback(const JointStateMsgPtr msg){
    has_joint_state = true;
    rt_joint_state_buffer.writeFromNonRT(msg);
}

void BipedController::contacts_callback(const ContactsMsgPtr msg){
    rt_contacts_buffer.writeFromNonRT(msg);
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn BipedController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

    std::string urdf_string;
    while (!this->get_parameter("robot_description", urdf_string))
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for parameter %s.", "/robot_description");
    while (!this->get_parameter("update_rate", update_rate))
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for parameter %s.", "/update_rate");
    if(update_rate <= 0){
        RCLCPP_FATAL(this->get_logger(), "Update Rate parameter is %i. Did you forget to set it?", update_rate);
        throw std::runtime_error("Invalid update_rate parameter");
    }

    // Integrator params
    integrate_from_current_state = params.integrator.from_current_state;

    RobotModelConfig robot_model_cfg;
    robot_model_cfg.file_or_string       = urdf_string;
    robot_model_cfg.submechanism_file    = params.robot_model.submechanism_file;  
    robot_model_cfg.floating_base        = params.robot_model.floating_base;  
    robot_model_cfg.joint_blacklist      = params.robot_model.joint_blacklist;
    auto contacts_param = params.robot_model.contacts;
    contacts.push_back(Contact(contacts_param.left_leg.name, contacts_param.left_leg.active, contacts_param.left_leg.mu));
    contacts.push_back(Contact(contacts_param.right_leg.name, contacts_param.right_leg.active, contacts_param.right_leg.mu));
    robot_model_cfg.contact_points = contacts;

    RCLCPP_INFO(this->get_logger(), "Configuring robot model: %s", params.robot_model.type.c_str());
    PluginLoader::loadPlugin("libwbc-robot_models-" + params.robot_model.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(params.robot_model.type));
    if(!robot_model->configure(robot_model_cfg)){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure robot model");
        return CallbackReturn::ERROR;
    }

    if(params.joint_names.empty()){        
        for(uint i = 0; i < robot_model->na(); i++)
            joint_idx_map.push_back(i);
    }
    else{
        assert(params.joint_names.size() == robot_model->na());
        for(uint i = 0; i < robot_model->na(); i++)
            joint_idx_map.push_back(robot_model->jointIndex(params.joint_names[i]));
    }

    // Gain scheduling params
    auto gain_sched_param = params.gain_scheduling;
    p_gain_stance = gain_sched_param.p_gain_stance;
    d_gain_stance = gain_sched_param.d_gain_stance;
    p_gain_swing = gain_sched_param.p_gain_swing;
    d_gain_swing = gain_sched_param.d_gain_swing;
    vector<string> jts_left_leg = gain_sched_param.joint_names_left_leg;
    vector<string> jts_right_leg = gain_sched_param.joint_names_right_leg;
    for(auto j : jts_left_leg)
        joint_idx_map_left_leg.push_back(robot_model->jointIndex(j));
    for(auto j : jts_right_leg)
        joint_idx_map_right_leg.push_back(robot_model->jointIndex(j));
    // Assume stance phase at startup
    p_gain = p_gain_stance; 
    d_gain = d_gain_stance;

    RCLCPP_INFO(this->get_logger(), "Configuring solver: %s", params.solver.type.c_str());
    PluginLoader::loadPlugin("libwbc-solvers-" + params.solver.type + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(params.solver.type));

    RCLCPP_INFO(this->get_logger(), "Configuring Tasks");
    vector<wbc::TaskPtr> tasks;

    // CoM Task
    auto com_param = params.tasks.com_position;                                                     
    com_task_iface = std::make_shared<CoMTaskInterface>("com_position", this->shared_from_this());
    com_task_iface->task = std::make_shared<CoMAccelerationTask>(TaskConfig("com_position", 0, com_param.weights, com_param.activation), 
                                                                 robot_model);
    Eigen::VectorXd p_gain(6), d_gain(6), max_ctrl_out(6);
    p_gain << com_param.p_gain[0], com_param.p_gain[1], com_param.p_gain[2], 0, 0, 0;
    d_gain << com_param.d_gain[0], com_param.d_gain[1], com_param.d_gain[2], 0, 0, 0;
    max_ctrl_out << com_param.max_control_output[0], com_param.max_control_output[1], com_param.max_control_output[2], 0, 0, 0;
    com_task_iface->controller.setPGain(p_gain);
    com_task_iface->controller.setDGain(d_gain);
    com_task_iface->controller.setMaxCtrlOutput(max_ctrl_out);
    tasks.push_back(com_task_iface->task);
    task_interfaces.push_back(com_task_iface);

    // Foot tasks
    auto foot_r_param = params.tasks.foot_r_pose;
    foot_r_iface = std::make_shared<FootTaskInterface>("foot_r_pose", this->shared_from_this());
    foot_r_iface->task = std::make_shared<SpatialAccelerationTask>(TaskConfig("foot_r_pose", 0, foot_r_param.weights, foot_r_param.activation), 
                                                                   robot_model,
                                                                   foot_r_param.tip_frame,
                                                                   foot_r_param.ref_frame); 
    foot_r_iface->controller.setPGain(Eigen::Map<Eigen::VectorXd>(foot_r_param.p_gain.data(), foot_r_param.p_gain.size()));
    foot_r_iface->controller.setDGain(Eigen::Map<Eigen::VectorXd>(foot_r_param.d_gain.data(), foot_r_param.d_gain.size()));
    foot_r_iface->controller.setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(foot_r_param.max_control_output.data(), foot_r_param.max_control_output.size()));
    tasks.push_back(foot_r_iface->task);
    task_interfaces.push_back(foot_r_iface);

    auto foot_l_param = params.tasks.foot_l_pose;
    foot_l_iface = std::make_shared<FootTaskInterface>("foot_l_pose", this->shared_from_this());
    foot_l_iface->task = std::make_shared<SpatialAccelerationTask>(TaskConfig("foot_l_pose", 0, foot_l_param.weights, foot_l_param.activation), 
                                                                   robot_model,
                                                                   foot_l_param.tip_frame,
                                                                   foot_l_param.ref_frame); 
    foot_l_iface->controller.setPGain(Eigen::Map<Eigen::VectorXd>(foot_l_param.p_gain.data(), foot_l_param.p_gain.size()));
    foot_l_iface->controller.setDGain(Eigen::Map<Eigen::VectorXd>(foot_l_param.d_gain.data(), foot_l_param.d_gain.size()));
    foot_l_iface->controller.setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(foot_l_param.max_control_output.data(), foot_l_param.max_control_output.size()));
    tasks.push_back(foot_l_iface->task);
    task_interfaces.push_back(foot_l_iface);

    // Contact force tasks
    auto force_l_param = params.tasks.contact_force_l;
    force_l_task = std::make_shared<ContactForceTask>(TaskConfig("foot_l_force", 0, force_l_param.weights, force_l_param.activation),
                                                      robot_model,
                                                      force_l_param.ref_frame);
    tasks.push_back(force_l_task);

    auto force_r_param = params.tasks.contact_force_r;
    force_r_task = std::make_shared<ContactForceTask>(TaskConfig("foot_r_force", 0, force_r_param.weights, force_r_param.activation),
                                                      robot_model,
                                                      force_r_param.ref_frame);
    tasks.push_back(force_r_task);

    // Joint position task
    auto joint_pos_param = params.tasks.joint_position;
    joint_pos_iface = std::make_shared<JointPositionTaskInterface>("joint_position", this->shared_from_this());
    joint_pos_iface->task = std::make_shared<JointAccelerationTask>(TaskConfig("joint_position", 0, joint_pos_param.weights, joint_pos_param.activation), 
                                                                    robot_model,
                                                                    params.joint_names); 
    joint_pos_iface->controller = std::make_shared<JointPosPDController>(params.joint_names.size());                                                                    
    joint_pos_iface->controller->setPGain(Eigen::Map<Eigen::VectorXd>(joint_pos_param.p_gain.data(), joint_pos_param.p_gain.size()));
    joint_pos_iface->controller->setDGain(Eigen::Map<Eigen::VectorXd>(joint_pos_param.d_gain.data(), joint_pos_param.d_gain.size()));
    joint_pos_iface->controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(joint_pos_param.max_control_output.data(), joint_pos_param.max_control_output.size()));
    tasks.push_back(joint_pos_iface->task);
    task_interfaces.push_back(joint_pos_iface);

    RCLCPP_INFO(this->get_logger(), "Configuring scene: %s", params.scene.type.c_str());
    PluginLoader::loadPlugin("libwbc-scenes-" + params.scene.type + ".so");
    scene = std::shared_ptr<Scene>(SceneFactory::createInstance(params.scene.type, robot_model, solver, 0.001));
    if(!scene->configure(tasks)){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure scene");
        return CallbackReturn::ERROR;
    } 

    joint_state.position.resize(robot_model->nj());
    joint_state.velocity.resize(robot_model->nj());
    joint_state.velocity.setZero();
    joint_state.acceleration.resize(robot_model->nj());
    joint_state.acceleration.setZero();

    // Subscribers/Publishers

    robot_state_subscriber = this->create_subscription<RobotStateMsg>("~/robot_state",
        rclcpp::SystemDefaultsQoS(), std::bind(&BipedController::robot_state_callback, this, placeholders::_1));

    joint_state_subscriber = this->create_subscription<JointStateMsg>("~/joint_state",
        rclcpp::SystemDefaultsQoS(), std::bind(&BipedController::joint_state_callback, this, placeholders::_1));

    joint_weight_subscriber = this->create_subscription<DoubleArrayMsg>("~/joint_weights", 
        rclcpp::SystemDefaultsQoS(), std::bind(&BipedController::joint_weight_callback, this, std::placeholders::_1));

    contacts_subscriber = this->create_subscription<ContactsMsg>("~/contacts", 
        rclcpp::SystemDefaultsQoS(), std::bind(&BipedController::contacts_callback, this, std::placeholders::_1));

    solver_output_publisher = this->create_publisher<JointCommandMsg>("~/solver_output", rclcpp::SystemDefaultsQoS());
    rt_solver_output_publisher = std::make_unique<RTJointCommandPublisher>(solver_output_publisher);

    timing_stats_publisher = this->create_publisher<TimingStatsMsg>("~/timing_stats", rclcpp::SystemDefaultsQoS());
    rt_timing_stats_publisher = std::make_unique<RTTimingStatsPublisher>(timing_stats_publisher);

    robot_model->setContacts(contacts);

    //  Timer to run control loop

    timer = this->create_wall_timer(std::chrono::milliseconds((int)1000.0/params.update_rate),std::bind(&BipedController::updateController, this));
    timer->cancel(); // Cancel here to start the timer on activate

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void BipedController::handleContacts(const ContactsMsgPtr msg){
    fromROS(*msg, contacts);
    assert(contacts.size() == 2);
    robot_model->setContacts(contacts);

    // 1. Schedule PD-gains based on the contacts
    // 2. Activate force tasks / deactivate position tasks for legs which are in contact.

    // Left Leg
    if(contacts[0].active){
        for(auto j : joint_idx_map_left_leg){
            p_gain[j] = p_gain_stance[j];
            d_gain[j] = d_gain_stance[j];
        }
        foot_l_iface->task->setActivation(0);
        force_l_task->setActivation(1);
        fromROS(msg->wrenches[0], contact_force_l);
        force_l_task->setReference(contact_force_l);
    }
     else{
        for(auto j : joint_idx_map_left_leg){
            p_gain[j] = p_gain_swing[j];
            d_gain[j] = d_gain_swing[j];
        }
        foot_l_iface->task->setActivation(1);
        force_l_task->setActivation(0);
    }

    // Right Leg
    if(contacts[1].active){
        for(auto j : joint_idx_map_right_leg){
            p_gain[j] = p_gain_stance[j];
            d_gain[j] = d_gain_stance[j];
        }
        foot_r_iface->task->setActivation(0);
        force_r_task->setActivation(1);
        fromROS(msg->wrenches[1], contact_force_r);
        force_r_task->setReference(contact_force_r);
    }
    else{
        for(auto j : joint_idx_map_right_leg){
            p_gain[j] = p_gain_swing[j];
            d_gain[j] = d_gain_swing[j];
        }
        foot_r_iface->task->setActivation(1);
        force_r_task->setActivation(0);
    }
}

void BipedController::updateController(){
     timing_stats.desired_period = 1.0/update_rate;
    if(stamp.nanoseconds() != 0)
        timing_stats.actual_period = (this->get_clock()->now() - stamp).seconds();
    stamp = this->get_clock()->now();

    if(!has_robot_state && !has_joint_state){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No robot state");
        return;
    }

    // 1. Update the internal robot model with the current robot state, floating base state, and contact information
    rclcpp::Time start = this->get_clock()->now();
    robot_state_msg = *rt_robot_state_buffer.readFromRT(); 
    joint_state_msg = *rt_joint_state_buffer.readFromRT();
    if(robot_state_msg.get()){
        fromROS(*robot_state_msg, joint_idx_map, joint_state, floating_base_state);
    }
    else if(joint_state_msg.get())
        fromROS(*joint_state_msg, joint_idx_map, joint_state);
    else
        return;
    robot_model->update(joint_state.position, joint_state.velocity, joint_state.acceleration,
                        floating_base_state.pose, floating_base_state.twist, floating_base_state.acceleration);

    contacts_msg = *rt_contacts_buffer.readFromRT();
    if(contacts_msg.get())
        handleContacts(contacts_msg);
    timing_stats.time_robot_model_update = (this->get_clock()->now() - start).seconds();

    // 2. Update Tasks
    start = this->get_clock()->now();
    for(auto ti : task_interfaces)
        ti->update(robot_model);

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

    // 5. Integrate and publish solution
    joint_integrator.integrate(robot_model->jointState(), solver_output, 1.0/update_rate, types::CommandMode::ACCELERATION, IntegrationMethod::RECTANGULAR, integrate_from_current_state);

    rt_solver_output_publisher->lock();
    toROS(solver_output, p_gain, d_gain, joint_idx_map, rt_solver_output_publisher->msg_);
    rt_solver_output_publisher->unlockAndPublish();

    timing_stats.time_per_cycle = (this->get_clock()->now() - stamp).seconds();
    rt_timing_stats_publisher->lock();
    rt_timing_stats_publisher->msg_ = timing_stats;
    rt_timing_stats_publisher->unlockAndPublish();
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn BipedController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    has_robot_state = has_joint_state = false;
    for(auto ti : task_interfaces)
        ti->reset();
    joint_integrator.reinit();
    timer->reset();

    RCLCPP_INFO(this->get_logger(), "Whole-Body Controller is running at %li Hz", params.update_rate);

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn BipedController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    timer->cancel();
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn BipedController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    task_interfaces.clear();
    joint_idx_map.clear();
    joint_idx_map_left_leg.clear();
    joint_idx_map_right_leg.clear();
    contacts.clear();

    com_task_iface.reset();
    foot_l_iface.reset();
    foot_r_iface.reset();
    force_l_task.reset();
    force_r_task.reset();
    joint_pos_iface.reset();
    robot_state_subscriber.reset();
    joint_state_subscriber.reset();
    timing_stats_publisher.reset();
    solver_output_publisher.reset();
    contacts_subscriber.reset();
    joint_weight_subscriber.reset();
    robot_model.reset();
    scene.reset();
    solver.reset();
    
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn BipedController::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn BipedController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(wbc_ros::BipedController)