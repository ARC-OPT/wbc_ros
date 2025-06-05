#include <wbc_ros/single_arm_controller.hpp>
#include <wbc_ros/conversions.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std;
using namespace wbc;

namespace wbc_ros{

SingleArmController::SingleArmController(const rclcpp::NodeOptions & options) : 
    rclcpp_lifecycle::LifecycleNode("whole_body_controller", options){
    param_listener = std::make_shared<single_arm_controller::ParamListener>(this->get_node_parameters_interface());
    params = param_listener->get_params();
}

void SingleArmController::joint_weight_callback(const JointWeightMsgPtr msg){
    rt_joint_weight_buffer.writeFromNonRT(msg);
}

void SingleArmController::robot_state_callback(const RobotStateMsgPtr msg){
    has_robot_state = true;
    rt_robot_state_buffer.writeFromNonRT(msg);
}

void SingleArmController::ee_pose_callback(const RigidBodyStateMsgPtr msg){
    rt_ee_pose_buffer.writeFromNonRT(msg);
}

void SingleArmController::joint_position_callback(const JointCommandMsgPtr msg){
    rt_joint_position_buffer.writeFromNonRT(msg);
}

void SingleArmController::elbow_pose_callback(const RigidBodyStateMsgPtr msg){
    rt_elbow_pose_buffer.writeFromNonRT(msg);
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn SingleArmController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
    std::string urdf_string;
    while (!this->get_parameter("robot_description", urdf_string))
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for parameter %s.", "/robot_description");
    while (!this->get_parameter("update_rate", update_rate))
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for parameter %s.", "/update_rate");
    if(update_rate <= 0){
        RCLCPP_FATAL(this->get_logger(), "Update Rate parameter is %i. Did you forget to set it?", update_rate);
        throw std::runtime_error("Invalid update_rate parameter");
    }

    RobotModelConfig robot_model_cfg;
    robot_model_cfg.file_or_string       = urdf_string;
    robot_model_cfg.submechanism_file    = params.robot_model.submechanism_file;  

    RCLCPP_INFO(this->get_logger(), "Configuring robot model: %s", params.robot_model.type.c_str());
    PluginLoader::loadPlugin("libwbc-robot_models-" + params.robot_model.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(params.robot_model.type));
    if(!robot_model->configure(robot_model_cfg)){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure robot model");
        return CallbackReturn::ERROR;
    }

    if(params.joint_names.empty()){        
        for(uint i = 0; i < robot_model->nj(); i++)
            joint_idx_map.push_back(i);
    }
    else{
        assert(params.joint_names.size() == robot_model->nj());
        for(uint i = 0; i < robot_model->nj(); i++)
            joint_idx_map.push_back(robot_model->jointIndex(params.joint_names[i]));
    }

    RCLCPP_INFO(this->get_logger(), "Configuring solver: %s", params.solver.type.c_str());
    PluginLoader::loadPlugin("libwbc-solvers-" + params.solver.type + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(params.solver.type));

    RCLCPP_INFO(this->get_logger(), "Configuring Tasks");
    vector<wbc::TaskPtr> tasks;
    // End effector pose
    auto ee_task_param = params.ee_pose_task;
    ee_pose_task = std::make_shared<wbc::SpatialVelocityTask>(TaskConfig("ee_pose", 0, ee_task_param.weights, ee_task_param.activation),
                                                              robot_model, ee_task_param.tip_frame, ee_task_param.ref_frame);
    tasks.push_back(ee_pose_task);
    ee_pose_controller = std::make_shared<wbc::CartesianPosPDController>();
    ee_pose_controller->setPGain(Eigen::Map<Eigen::VectorXd>(ee_task_param.p_gain.data(), ee_task_param.p_gain.size()));
    ee_pose_controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(ee_task_param.max_control_output.data(), ee_task_param.max_control_output.size())); 
    
    // Joint position
    auto jnt_task_param = params.joint_position_task;
    joint_position_task = std::make_shared<wbc::JointVelocityTask>(TaskConfig("joint_position", 0, jnt_task_param.weights, jnt_task_param.activation),
                                                                              robot_model, robot_model->jointNames());
    tasks.push_back(joint_position_task);                                           
    joint_pos_controller = std::make_shared<wbc::JointPosPDController>(robot_model->nj());
    joint_pos_controller->setPGain(Eigen::Map<Eigen::VectorXd>(jnt_task_param.p_gain.data(), jnt_task_param.p_gain.size()));
    joint_pos_controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(jnt_task_param.max_control_output.data(), jnt_task_param.max_control_output.size())); 

    // Elbow posture
    auto elbow_task_param = params.elbow_pose_task;
    elbow_pose_task = std::make_shared<wbc::SpatialVelocityTask>(TaskConfig("elbow_pose", 0, elbow_task_param.weights, elbow_task_param.activation),
                                                                          robot_model, elbow_task_param.tip_frame, elbow_task_param.ref_frame);
    tasks.push_back(elbow_pose_task);                                           
    elbow_pose_controller = std::make_shared<wbc::CartesianPosPDController>();
    elbow_pose_controller->setPGain(Eigen::Map<Eigen::VectorXd>(elbow_task_param.p_gain.data(), elbow_task_param.p_gain.size()));
    elbow_pose_controller->setMaxCtrlOutput(Eigen::Map<Eigen::VectorXd>(elbow_task_param.max_control_output.data(), elbow_task_param.max_control_output.size())); 

    RCLCPP_INFO(this->get_logger(), "Configuring scene: velocity_qp");
    PluginLoader::loadPlugin("libwbc-scenes-velocity_qp.so");
    scene = std::shared_ptr<Scene>(SceneFactory::createInstance("velocity_qp", robot_model, solver, 0.001));
    if(!scene->configure(tasks)){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure scene");
        return CallbackReturn::ERROR;
    } 

    // Subscribers/Publishers

    robot_state_subscriber = this->create_subscription<RobotStateMsg>("~/robot_state",
        rclcpp::SystemDefaultsQoS(), std::bind(&SingleArmController::robot_state_callback, this, placeholders::_1));

    ee_pose_subscriber = this->create_subscription<RigidBodyStateMsg>("~/ee_pose/setpoint",
        rclcpp::SystemDefaultsQoS(), std::bind(&SingleArmController::ee_pose_callback, this, placeholders::_1));

    joint_position_subscriber = this->create_subscription<JointCommandMsg>("~/joint_position/setpoint",
        rclcpp::SystemDefaultsQoS(), std::bind(&SingleArmController::joint_position_callback, this, placeholders::_1));

    elbow_pose_subscriber = this->create_subscription<RigidBodyStateMsg>("~/elbow_pose/setpoint",
        rclcpp::SystemDefaultsQoS(), std::bind(&SingleArmController::elbow_pose_callback, this, placeholders::_1));

    joint_weight_subscriber = this->create_subscription<JointWeightMsg>("~/joint_weights", 
        rclcpp::SystemDefaultsQoS(), std::bind(&SingleArmController::joint_weight_callback, this, std::placeholders::_1));

    solver_output_publisher = this->create_publisher<CommandMsg>("~/solver_output", rclcpp::SystemDefaultsQoS());
    rt_solver_output_publisher = std::make_unique<RTCommandPublisher>(solver_output_publisher);

    timing_stats_publisher = this->create_publisher<TimingStatsMsg>("~/timing_stats", rclcpp::SystemDefaultsQoS());
    rt_timing_stats_publisher = std::make_unique<RTTimingStatsPublisher>(timing_stats_publisher);

    // Timer to run control loop

    timer = this->create_wall_timer(std::chrono::milliseconds((int)1000.0/params.update_rate),std::bind(&SingleArmController::updateController, this));
    timer->cancel(); // Cancel here to start the timer on activate

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn SingleArmController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
    has_robot_state = false;
    ee_pose_task->reset();
    joint_position_task->reset();
    elbow_pose_task->reset();
    timer->reset();
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

void SingleArmController::updateController(){
    timing_stats.desired_period = 1.0/update_rate;
    if(stamp.nanoseconds() != 0)
        timing_stats.actual_period = (this->get_clock()->now() - stamp).seconds();
    stamp = this->get_clock()->now();

    if(!has_robot_state){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No robot state");
        return;
    }

    // 1. Update the internal robot model with the current robot state
    // Update joint state
    rclcpp::Time start = this->get_clock()->now();
    robot_state_msg = *rt_robot_state_buffer.readFromRT();   
    if(robot_state_msg->joint_state.position.size() > 0)
        fromROS(robot_state_msg->joint_state, joint_idx_map, joint_state);
    else
        return;
    robot_model->update(joint_state.position, joint_state.velocity, joint_state.acceleration);

    // 2. Update Tasks
    start = this->get_clock()->now();
    // EE Pose task
    ee_pose_msg = *rt_ee_pose_buffer.readFromRT(); 
    if(ee_pose_msg.get()){
        fromROS(*ee_pose_msg, ee_pose);
        ee_pose_task->setReference(ee_pose_controller->update(ee_pose.pose, 
                                                              ee_pose.twist, 
                                                              robot_model->pose(ee_pose_task->tipFrame())));
    }
    // Joint position task
    joint_position_msg = *rt_joint_position_buffer.readFromRT(); 
    if(joint_position_msg.get()){
        fromROS(*joint_position_msg, joint_idx_map, joint_position);
        joint_position_task->setReference(joint_pos_controller->update(joint_position.position,
                                                                       joint_position.velocity,
                                                                       robot_model->jointState().position));
    }
    // Elbow pose task
    elbow_pose_msg = *rt_elbow_pose_buffer.readFromRT(); 
    if(elbow_pose_msg.get()){
        fromROS(*elbow_pose_msg, elbow_pose);
        elbow_pose_task->setReference(elbow_pose_controller->update(elbow_pose.pose, 
                                                              elbow_pose.twist, 
                                                              robot_model->pose(elbow_pose_task->tipFrame())));
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

    // 5. Integrate and publish solution
    joint_integrator.integrate(robot_model->jointState(), solver_output, 1.0/update_rate, types::CommandMode::VELOCITY);

    rt_solver_output_publisher->lock();    
    toROS(solver_output, joint_idx_map, rt_solver_output_publisher->msg_);
    rt_solver_output_publisher->unlockAndPublish();

    timing_stats.time_per_cycle = (this->get_clock()->now() - stamp).seconds();
    rt_timing_stats_publisher->lock();
    rt_timing_stats_publisher->msg_ = timing_stats;
    rt_timing_stats_publisher->unlockAndPublish();
}


rclcpp_lifecycle::LifecycleNode::CallbackReturn SingleArmController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
    timer->cancel(); 
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn SingleArmController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    joint_state.clear();
    solver_output.clear();
    joint_idx_map.clear();
    joint_integrator.reinit();

    PluginLoader::unloadPlugin("libwbc-robot_models-" + params.robot_model.type + ".so");
    PluginLoader::unloadPlugin("libwbc-solvers-" + params.solver.type + ".so");
    PluginLoader::unloadPlugin("libwbc-scenes-velocity_qp.so");

    RobotModelFactory::clear();
    QPSolverFactory::clear();
    SceneFactory::clear();

    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn SingleArmController::on_error(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn SingleArmController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(wbc_ros::SingleArmController)