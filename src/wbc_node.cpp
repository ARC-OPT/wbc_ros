#include "wbc_node.hpp"
#include <wbc/scenes/VelocitySceneQuadraticCost.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/core/RobotModelFactory.hpp>
#include <wbc/core/QPSolverFactory.hpp>
#include "conversions.hpp"

using namespace std;
using namespace wbc;
using namespace rclcpp;

WbcNode::WbcNode(string node_name) : ControllerNode(node_name), has_floating_base_state(false){

    declare_parameter("integrate", true);
    integrate = get_parameter("integrate").as_bool();

    RobotModelConfig robot_model_cfg;
    string r = "robot_model_config.";
    declare_parameter(r + "file", robot_model_cfg.file);
    declare_parameter(r + "type", robot_model_cfg.type);
    declare_parameter(r + "submechanism_file", robot_model_cfg.submechanism_file);
    declare_parameter(r + "floating_base", robot_model_cfg.floating_base);
    declare_parameter(r + "contact_points.names", robot_model_cfg.contact_points.names);
    robot_model_cfg.file                 = get_parameter(r + "file").as_string();
    robot_model_cfg.type                 = get_parameter(r + "type").as_string();
    robot_model_cfg.submechanism_file    = get_parameter(r + "submechanism_file").as_string();
    robot_model_cfg.floating_base        = get_parameter(r + "floating_base").as_bool();
    robot_model_cfg.contact_points.names = get_parameter(r + "contact_points.names").as_string_array();
    // ROS2 does not support lists of complex data types as parameters, so we have to refer to this ugly hack here
    for(auto n : robot_model_cfg.contact_points.names){
        string rcn = "robot_model_config.contact_points." + n;
        declare_parameter(rcn + ".active", ActiveContact().active);
        declare_parameter(rcn + ".mu",     ActiveContact().mu);
        declare_parameter(rcn + ".wx",     ActiveContact().wx);
        declare_parameter(rcn + ".wy",     ActiveContact().wy);
        robot_model_cfg.contact_points[n].active = get_parameter(rcn + ".active").as_int();
        robot_model_cfg.contact_points[n].mu     = get_parameter(rcn + ".mu").as_double();
        robot_model_cfg.contact_points[n].wx     = get_parameter(rcn + ".wx").as_double();
        robot_model_cfg.contact_points[n].wy     = get_parameter(rcn + ".wy").as_double();
    }

    RCLCPP_INFO(get_logger(), "Configuring robot model: %s", robot_model_cfg.type.c_str());
    PluginLoader::loadPlugin("libwbc-robot_models-" + robot_model_cfg.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(robot_model_cfg.type));
    if(!robot_model->configure(robot_model_cfg)){
        RCLCPP_ERROR(get_logger(), "Failed to configure robot model");
        abort();
    }

    declare_parameter("qp_solver", "qpoases");
    std::string qp_solver = get_parameter("qp_solver").as_string();

    RCLCPP_INFO(get_logger(), "Configuring solver: %s", qp_solver.c_str());
    PluginLoader::loadPlugin("libwbc-solvers-" + qp_solver + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(qp_solver));

    vector<TaskConfig> wbc_config;
    declare_parameter("wbc_config.names", vector<string>());
    vector<string> task_names = get_parameter("wbc_config.names").as_string_array();
    for(auto n : task_names){
        TaskConfig cfg;
        string wcn = "wbc_config." + n;
        declare_parameter(wcn + ".priority", cfg.priority);
        declare_parameter(wcn + ".type", (int)cfg.type);
        declare_parameter(wcn + ".root", cfg.root);
        declare_parameter(wcn + ".tip", cfg.tip);
        declare_parameter(wcn + ".ref_frame", cfg.ref_frame);
        declare_parameter(wcn + ".weights", cfg.weights);
        declare_parameter(wcn + ".joint_names", cfg.joint_names);
        declare_parameter(wcn + ".activation", cfg.activation);
        declare_parameter(wcn + ".timeout", cfg.timeout);
        cfg.name = n;
        cfg.priority    = get_parameter(wcn + ".priority").as_int();
        cfg.type        = (TaskType)get_parameter(wcn + ".type").as_int();
        cfg.root        = get_parameter(wcn + ".root").as_string();
        cfg.tip         = get_parameter(wcn + ".tip").as_string();
        cfg.ref_frame   = get_parameter(wcn + ".ref_frame").as_string();
        cfg.weights     = get_parameter(wcn + ".weights").as_double_array();
        cfg.joint_names = get_parameter(wcn + ".joint_names").as_string_array();
        cfg.activation  = get_parameter(wcn + ".activation").as_double();
        cfg.timeout     = get_parameter(wcn + ".timeout").as_double();
        wbc_config.push_back(cfg);
    }

    RCLCPP_INFO(get_logger(), "Configuring scene: %s", "VelocitySceneQuadraticCost");
    scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    if(!scene->configure(wbc_config)){
        RCLCPP_ERROR(get_logger(), "Failed to configure scene");
        abort();
    }

    RCLCPP_INFO(get_logger(), "Creating interfaces");

    // Input joint state
    joint_state.resize(robot_model->noOfJoints());
    joint_state.names = robot_model->jointNames();
    sub_feedback = create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, bind(&WbcNode::jointStateCallback, this, placeholders::_1));

    // Input floating base state
    if(robot_model_cfg.floating_base)
        sub_floating_base = create_subscription<wbc_msgs::msg::RigidBodyState>("floating_base_state", 1, bind(&WbcNode::floatingBaseStateCallback, this, placeholders::_1));

    // Input references, task weights and activations
    for(auto w : wbc_config){
        if(w.type == cart){
            function<void(const wbc_msgs::msg::RigidBodyState& msg)> fcn = bind(&WbcNode::cartReferenceCallback, this, placeholders::_1, w.name);
            sub_cart_ref.push_back(create_subscription<wbc_msgs::msg::RigidBodyState>("ref_" + w.name, 1, fcn));
        }
        else{
            function<void(const trajectory_msgs::msg::JointTrajectory& msg)> fcn = bind(&WbcNode::jntReferenceCallback, this, placeholders::_1, w.name);
            sub_jnt_ref.push_back(create_subscription<trajectory_msgs::msg::JointTrajectory>("ref_" + w.name, 1, fcn));
        }
        function<void(const std_msgs::msg::Float64MultiArray& msg)> fcn = bind(&WbcNode::taskWeightsCallback, this, placeholders::_1, w.name);
        sub_weights.push_back(create_subscription<std_msgs::msg::Float64MultiArray>("weights_" + w.name, 1, fcn));

        function<void(const std_msgs::msg::Float64& msg)> fcn2 = bind(&WbcNode::taskActivationCallback, this, placeholders::_1, w.name);
        sub_activation.push_back(create_subscription<std_msgs::msg::Float64>("activation_" + w.name, 1, fcn2));
    }

    // Output task states
    for(auto w : wbc_config){
        if(w.type == cart)
            publishers_task_status_cart.push_back(create_publisher<wbc_msgs::msg::RigidBodyState>("status_" + w.name, 1));
        else
            publishers_task_status_jnt.push_back(create_publisher<sensor_msgs::msg::JointState>("status_" + w.name, 1));
    }

    // Output task info
    for(auto w : wbc_config)
        publishers_task_info.push_back(create_publisher<wbc_msgs::msg::TaskStatus>("task_" + w.name, 1));

    // Input joint weights
    sub_joint_weights = create_subscription<std_msgs::msg::Float64MultiArray>("joint_weights", 1, bind(&WbcNode::jointWeightsCallback, this, placeholders::_1));

    // Stats on computation time
    pub_timing_stats = create_publisher<wbc_msgs::msg::WbcTimingStats>("timing_stats", 1);

    // Solver output
    solver_output_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("solver_output", 1);

    // WBC should send zeros if no setpoint is given from any controller, so set has_setpoint = true
    has_setpoint = true;
    stamp = get_clock()->now();

    RCLCPP_INFO(get_logger(), "WBC Node is running");
}

WbcNode::~WbcNode(){
}

void WbcNode::jointStateCallback(const sensor_msgs::msg::JointState& msg){
    fromROS(msg,joint_state);
    if(robot_model->hasFloatingBase()){
        if(has_floating_base_state)
            has_feedback = true;
    }
    else
        has_feedback = true;
}

void WbcNode::cartReferenceCallback(const wbc_msgs::msg::RigidBodyState& msg, const string& constraint_name){
    fromROS(msg, reference_cart);
    scene->setReference(constraint_name, reference_cart);
}

void WbcNode::jntReferenceCallback(const trajectory_msgs::msg::JointTrajectory& msg, const string& constraint_name){
    fromROS(msg, reference_jnt);
    scene->setReference(constraint_name, reference_jnt);
}

void WbcNode::taskActivationCallback(const std_msgs::msg::Float64& msg, const string& constraint_name){
    scene->setTaskActivation(constraint_name, msg.data);
}

void WbcNode::taskWeightsCallback(const std_msgs::msg::Float64MultiArray& msg, const string& constraint_name){
    fromROS(msg, task_weights);
    scene->setTaskWeights(constraint_name, task_weights);
}

void WbcNode::jointWeightsCallback(const std_msgs::msg::Float64MultiArray& msg){
    fromROS(msg, robot_model->jointNames(), joint_weights);
    scene->setJointWeights(joint_weights);
}

void WbcNode::floatingBaseStateCallback(const wbc_msgs::msg::RigidBodyState& msg){
    fromROS(msg, floating_base_state);
    has_floating_base_state = true;
}

void WbcNode::updateController(){
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
    toROS(solver_output, solver_output_ros);
    solver_output_publisher->publish(solver_output_ros);


    publishTaskStatus();
    publishTaskInfo();

    timing_stats.time_per_cycle = (get_clock()->now() - stamp).seconds();
    timing_stats.header.stamp = get_clock()->now();
    pub_timing_stats->publish(timing_stats);
}

void WbcNode::publishTaskStatus(){
    for(uint i = 0; i < wbc_config.size(); i++){
        /*const TaskConfig& w = wbc_config[i];
        if(w.type == cart){
            //toROS(robot_model->rigidBodyState(w.ref_frame, w.tip), status_cart);
            //publishers_task_status[i].publish(status_cart);
        }
        else{
            //toROS(robot_model->jointState(w.joint_names), status_jnt);
            //publishers_task_status[i].publish(status_jnt);
        }*/
    }
}

void WbcNode::publishTaskInfo(){
    task_status_msgs.resize(tasks_status.size());
    for(uint i = 0; i < tasks_status.size(); i++){
        /*const TaskStatus& w = tasks_status[i];
        toROS(w, task_status_msgs[i]);
        publishers_task_info[i].publish(task_status_msgs[i]);*/
    }
}

int main(int argc, char** argv){
    init(argc, argv);
    spin(make_shared<WbcNode>("wbc"));
}
