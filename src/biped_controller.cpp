#include <wbc_ros/biped_controller.hpp>
#include <wbc/core/PluginLoader.hpp>
#include <sstream>

using namespace std;
using namespace wbc;
using namespace wbc::types;

BipedController::BipedController(const string& name, const rclcpp::NodeOptions & options) : 
    rclcpp_lifecycle::LifecycleNode("whole_body_control", options){
    param_listener = std::make_shared<whole_body_control::ParamListener>(this->get_node_parameters_interface());
}

void BipedController::robot_state_callback(const RobotStateMsgPtr msg){
    has_robot_state = true;
    solver_output_msg.stamp = msg->stamp;
    fromROS(*msg, joint_idx_map, joint_state, floating_base_state);
}

void BipedController::setpoint_callback(const WbcControlTargetMsgPtr msg){
    has_setpoint = true;
    control_target_msg = *msg;
}

void BipedController::set_dynamic_params(whole_body_control::Params params){

    RCLCPP_INFO_STREAM(this->get_logger(), "Setting dynamic parameters for WBC");

    auto contacts_param = params.robot_model.contacts;
    contacts[0].mu = contacts_param.left_leg.mu;
    contacts[0].wx = contacts_param.left_leg.wx;
    contacts[0].wy = contacts_param.left_leg.wy;
    contacts[1].mu = contacts_param.right_leg.mu;
    contacts[1].wx = contacts_param.right_leg.wx;
    contacts[1].wy = contacts_param.right_leg.wy;
    robot_model->setContacts(contacts);
    robot_model->setJointWeights(sortEigen(params.joint_weights, joint_idx_map));

    solver_output_msg.kp = std::vector<double>(params.p_gain.data(), params.p_gain.data() + params.p_gain.size());
    solver_output_msg.kd = std::vector<double>(params.d_gain.data(), params.d_gain.data() + params.d_gain.size());

    auto body_param = params.tasks.body_pose;   
    body_task_iface.task->setWeights(Eigen::Map<Eigen::VectorXd>(body_param.weights.data(), body_param.weights.size()));
    body_task_iface.controller.setPGain(Eigen::Map<Eigen::VectorXd>(body_param.p_gain.data(), body_param.p_gain.size()));
    body_task_iface.controller.setDGain(Eigen::Map<Eigen::VectorXd>(body_param.d_gain.data(), body_param.d_gain.size()));

    auto foot_l_param = params.tasks.foot_l_pose;
    foot_l_iface.task->setWeights(Eigen::Map<Eigen::VectorXd>(foot_l_param.weights.data(), foot_l_param.weights.size()));
    foot_l_iface.controller.setPGain(Eigen::Map<Eigen::VectorXd>(foot_l_param.p_gain.data(), foot_l_param.p_gain.size()));
    foot_l_iface.controller.setDGain(Eigen::Map<Eigen::VectorXd>(foot_l_param.d_gain.data(), foot_l_param.d_gain.size()));

    auto foot_r_param = params.tasks.foot_r_pose;
    foot_r_iface.task->setWeights(Eigen::Map<Eigen::VectorXd>(foot_r_param.weights.data(), foot_r_param.weights.size()));
    foot_r_iface.controller.setPGain(Eigen::Map<Eigen::VectorXd>(foot_r_param.p_gain.data(), foot_r_param.p_gain.size()));
    foot_r_iface.controller.setDGain(Eigen::Map<Eigen::VectorXd>(foot_r_param.d_gain.data(), foot_r_param.d_gain.size()));

    auto force_l_param = params.tasks.contact_force_l;
    force_l_task->setWeights(Eigen::Map<Eigen::VectorXd>(force_l_param.weights.data(), force_l_param.weights.size()));

    auto force_r_param = params.tasks.contact_force_r;
    force_r_task->setWeights(Eigen::Map<Eigen::VectorXd>(force_r_param.weights.data(), force_r_param.weights.size()));

    auto joint_pos_param = params.tasks.joint_position;    
    joint_pos_iface.task->setWeights(sortEigen(joint_pos_param.weights, joint_idx_map));
    joint_pos_iface.controller->setPGain(sortEigen(joint_pos_param.p_gain, joint_idx_map));
    joint_pos_iface.controller->setDGain(sortEigen(joint_pos_param.d_gain, joint_idx_map));
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn BipedController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

    params = param_listener->get_params();
    param_listener->setUserCallback([this](const auto& params) { set_dynamic_params(params); });

    string urdf_string;
    while (!this->get_parameter("robot_description", urdf_string))
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for parameter %s.", "/robot_description");
    while (!this->get_parameter("update_rate", update_rate))
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for parameter %s.", "/update_rate");
    if(update_rate <= 0){
        RCLCPP_FATAL(this->get_logger(), "Update Rate parameter is %i. Did you forget to set it?", update_rate);
        throw runtime_error("Invalid update_rate parameter");
    }

    RCLCPP_INFO(this->get_logger(), "Configuring robot model: %s", params.robot_model.type.c_str());

    RobotModelConfig robot_model_cfg;
    robot_model_cfg.file_or_string              = urdf_string;
    robot_model_cfg.floating_base               = true;
    auto contacts_param = params.robot_model.contacts;
    contacts.push_back(Contact(contacts_param.left_leg.name, 1.0, contacts_param.left_leg.mu, contacts_param.left_leg.wx, contacts_param.left_leg.wy));
    contacts.push_back(Contact(contacts_param.right_leg.name, 1.0, contacts_param.right_leg.mu, contacts_param.right_leg.wx, contacts_param.right_leg.wy));
    robot_model_cfg.contact_points = contacts;

    PluginLoader::loadPlugin("libwbc-robot_models-" + params.robot_model.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(params.robot_model.type));
    if(!robot_model->configure(robot_model_cfg)){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure robot model");
        return CallbackReturn::ERROR;
    }

    for(uint i = 0; i < robot_model->na(); i++)
        joint_idx_map.push_back(robot_model->jointIndex(params.joint_names[i]));    

    {
        std::ostringstream oss;
        oss << "WBC Joint Names internal (pinocchio): ";
        for(auto n : robot_model->jointNames()) oss << n << ", ";
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }
    {
        std::ostringstream oss;
        oss << "WBC Joint Names external: ";
        for(auto n : params.joint_names) oss << n << ", ";
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }
    {
        std::ostringstream oss;
        oss << "WBC joint index map: ";
        for(auto j : joint_idx_map) oss << j << ", ";
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Configuring solver: %s", params.solver.type.c_str());
    PluginLoader::loadPlugin("libwbc-solvers-" + params.solver.type + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(params.solver.type));

    RCLCPP_INFO(this->get_logger(), "Configuring Tasks");
    vector<wbc::TaskPtr> tasks;

    // Body Pose Task
    string task_name = "body_pose";
    RCLCPP_INFO(this->get_logger(), "Task %s", task_name.c_str());
    auto body_param = params.tasks.body_pose;       
    Eigen::VectorXd body_weights = Eigen::Map<Eigen::VectorXd>(body_param.weights.data(), body_param.weights.size());                      
    body_task_iface.task = std::make_shared<SpatialAccelerationTask>(TaskConfig(task_name, 0, body_weights, 1.0),
                                                                     robot_model,
                                                                     body_param.tip_frame); 
    tasks.push_back(body_task_iface.task);

    // Foot tasks
    task_name = "foot_l_pose";
    RCLCPP_INFO(this->get_logger(), "Task %s", task_name.c_str());
    auto foot_l_param = params.tasks.foot_l_pose;
    Eigen::VectorXd foot_l_weights = Eigen::Map<Eigen::VectorXd>(foot_l_param.weights.data(), foot_l_param.weights.size());   
    foot_l_iface.task = std::make_shared<SpatialAccelerationTask>(TaskConfig(task_name, 0, foot_l_weights, 1.0), 
                                                                  robot_model,
                                                                  foot_l_param.tip_frame); 
    tasks.push_back(foot_l_iface.task);

    task_name = "foot_r_pose";
    RCLCPP_INFO(this->get_logger(), "Task %s", task_name.c_str());
    auto foot_r_param = params.tasks.foot_r_pose;
    Eigen::VectorXd foot_r_weights = Eigen::Map<Eigen::VectorXd>(foot_r_param.weights.data(), foot_r_param.weights.size());
    foot_r_iface.task = std::make_shared<SpatialAccelerationTask>(TaskConfig(task_name, 0, foot_r_weights, 1.0),
                                                                  robot_model,
                                                                  foot_r_param.tip_frame); 
    tasks.push_back(foot_r_iface.task);

    // Contact force tasks
    task_name = "foot_l_force";
    RCLCPP_INFO(this->get_logger(), "Task %s", task_name.c_str());
    auto force_l_param = params.tasks.contact_force_l;
    Eigen::VectorXd force_l_weights = Eigen::Map<Eigen::VectorXd>(force_l_param.weights.data(), force_l_param.weights.size());
    force_l_task = std::make_shared<ContactWrenchTask>(TaskConfig(task_name, 0, force_l_weights, 1.0),
                                                       robot_model);
    tasks.push_back(force_l_task);

    task_name = "foot_r_force";
    auto force_r_param = params.tasks.contact_force_r;
    Eigen::VectorXd force_r_weights = Eigen::Map<Eigen::VectorXd>(force_r_param.weights.data(), force_r_param.weights.size());
    force_r_task = std::make_shared<ContactWrenchTask>(TaskConfig(task_name, 0, force_r_weights, 1.0),
                                                       robot_model);
    tasks.push_back(force_r_task);

    // Joint position task
    task_name = "joint_position";
    RCLCPP_INFO(this->get_logger(), "Task %s", task_name.c_str());
    auto joint_pos_param = params.tasks.joint_position;    
    Eigen::VectorXd joint_position_weights = sortEigen(joint_pos_param.weights, joint_idx_map);
    joint_pos_iface.task = std::make_shared<JointAccelerationTask>(TaskConfig(task_name, 0, joint_position_weights, 1.0), 
                                                                   robot_model,
                                                                   params.joint_names); 
    joint_pos_iface.controller = std::make_shared<JointPosPDController>(params.joint_names.size());
    tasks.push_back(joint_pos_iface.task);

    RCLCPP_INFO(this->get_logger(), "Configuring scene: AccelerationSceneReducedTSID");
    scene = std::make_shared<AccelerationSceneReducedTSID>(robot_model, solver, 1.0/params.update_rate, (uint)6);
    if(!scene->configure(tasks)){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure scene");
        return CallbackReturn::ERROR;
    } 

    set_dynamic_params(params);

    joint_state.position.resize(robot_model->na());
    joint_state.velocity.resize(robot_model->na());
    joint_state.velocity.setZero();
    joint_state.acceleration.resize(robot_model->na());
    joint_state.acceleration.setZero();

    solver_output.position.resize(robot_model->na());
    solver_output.position.setZero();
    solver_output.velocity.resize(robot_model->na());
    solver_output.velocity.setZero();
    solver_output.acceleration.resize(robot_model->na());
    solver_output.acceleration.setZero();
    solver_output.effort.resize(robot_model->na());
    solver_output.effort.setZero();

    solver_output_act_msg.position.resize(robot_model->na());
    solver_output_act_msg.velocity.resize(robot_model->na());
    solver_output_act_msg.acceleration.resize(robot_model->na());
    solver_output_act_msg.effort.resize(robot_model->na());

    
    // Subscribers/Publishers

    robot_state_subscriber = this->create_subscription<RobotStateMsg>("~/robot_state",
        QOS_BEST_EFFORT_NO_DEPTH, bind(&BipedController::robot_state_callback, this, placeholders::_1));

    control_target_subscriber = this->create_subscription<WbcControlTargetMsg>("~/setpoint", 
        QOS_BEST_EFFORT_NO_DEPTH, bind(&BipedController::setpoint_callback, this, placeholders::_1));

    solver_output_publisher = this->create_publisher<JointCommandMsg>("~/solver_output", QOS_RELIABLE_NO_DEPTH);
    timing_stats_publisher = this->create_publisher<TimingStatsMsg>("~/timing_ms", QOS_BEST_EFFORT_NO_DEPTH);

    //  Timer to run control loop

    timer = rclcpp::create_timer(this, this->get_clock(), chrono::milliseconds((int)1000.0/params.update_rate), bind(&BipedController::updateController, this));
    timer->cancel(); // Cancel here to start the timer on activate

    RCLCPP_INFO(this->get_logger(), "BipedController configured successfully");

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void BipedController::handleContacts(const ContactsMsg& msg){
    fromROS(msg, contacts);
    assert(contacts.size() == 2);
    robot_model->setContacts(contacts);

    // 1. Schedule PD-gains based on the contacts
    // 2. Activate force tasks / deactivate position tasks for legs which are in contact.

    // Left Leg
    if(contacts[0].active){
        foot_l_iface.task->setActivation(0);
        force_l_task->setActivation(1);
        fromROS(msg.wrenches[0], contact_force_l);
        force_l_task->setReference(contact_force_l);
    }
     else{
        foot_l_iface.task->setActivation(1);
        force_l_task->setActivation(0);
    }

    // Right Leg
    if(contacts[1].active){
        foot_r_iface.task->setActivation(0);
        force_r_task->setActivation(1);
        fromROS(msg.wrenches[1], contact_force_r);
        force_r_task->setReference(contact_force_r);
    }
    else{
        foot_r_iface.task->setActivation(1);
        force_r_task->setActivation(0);
    }
}

void BipedController::updateController(){
    // Do not do anything if there is no robot state
    if(!has_robot_state){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No robot state received");
        return;
    }

    rclcpp::Time start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    timing_stats.desired_period = 1000.0/update_rate;
    if(stamp.nanoseconds() != 0)
        timing_stats.actual_period = (rclcpp::Clock(RCL_SYSTEM_TIME).now() - stamp).seconds()*1000.0;
    stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    // 1. Update the internal robot model with the current robot state, floating base state, and contact information
    rclcpp::Time start = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    robot_model->update(joint_state.position, joint_state.velocity, joint_state.acceleration,
                        floating_base_state.pose, floating_base_state.twist, floating_base_state.acceleration);        
    timing_stats.time_robot_model_update = (rclcpp::Clock(RCL_SYSTEM_TIME).now() - start).seconds()*1000.0;


    // Only compute control output if there is a setpoint
    if(!has_setpoint){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No WBC control target received");
        return;
    }

    // 2. Update Tasks and contacts
    start = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    handleContacts(control_target_msg.contacts);
    body_task_iface.update(robot_model, control_target_msg.body_pose_setpoint);
    foot_l_iface.update(robot_model, control_target_msg.left_foot_setpoint);    
    foot_r_iface.update(robot_model, control_target_msg.right_foot_setpoint);    
    joint_pos_iface.update(robot_model, control_target_msg.joint_position_setpoint, joint_idx_map);

    // 3. Set up QP
    qp = scene->update();
    timing_stats.time_scene_update = (rclcpp::Clock(RCL_SYSTEM_TIME).now() - start).seconds()*1000.0;

    // 4. Solve the QP
    start = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    solver_output = scene->solve(qp);    
    timing_stats.time_solve = (rclcpp::Clock(RCL_SYSTEM_TIME).now() - start).seconds()*1000.0;

    // 5. Integrate and publish solution (independt joints)
    joint_integrator.integrate(robot_model->jointState(), solver_output, 1.0/update_rate, types::CommandMode::ACCELERATION, IntegrationMethod::RECTANGULAR, true);

    toROS(solver_output, joint_idx_map, solver_output_msg);
    solver_output_publisher->publish(solver_output_msg);

    timing_stats.time_per_cycle = (rclcpp::Clock(RCL_SYSTEM_TIME).now() - stamp).seconds()*1000.0;
    timing_stats_publisher->publish(timing_stats);
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn BipedController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    has_robot_state = has_setpoint = false;
    body_task_iface.task->reset();
    foot_l_iface.task->reset();
    foot_r_iface.task->reset();
    joint_pos_iface.task->reset();
    
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
    joint_idx_map.clear();
    contacts.clear();

    robot_state_subscriber.reset();
    timing_stats_publisher.reset();
    solver_output_publisher.reset();
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

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<BipedController>("whole_body_control");
	rclcpp::spin(node->get_node_base_interface());
	rclcpp::shutdown();
	return 0;
}
