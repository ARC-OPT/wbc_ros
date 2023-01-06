#include "wbc_node.hpp"
#include <wbc/scenes/VelocitySceneQuadraticCost.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/core/RobotModelFactory.hpp>
#include <wbc/core/QPSolverFactory.hpp>
#include "conversions.hpp"

using namespace std;
using namespace wbc;

WbcNode::WbcNode(int argc, char** argv) : ControllerNode(argc, argv), has_floating_base_state(false){

    checkParam("integrate");
    ros::param::get("integrate", integrate);

    checkParam("robot_model_config");
    XmlRpc::XmlRpcValue xml_rpc_val;
    ros::param::get("robot_model_config", xml_rpc_val);
    RobotModelConfig robot_model_cfg;
    fromROS(xml_rpc_val, robot_model_cfg);

    ROS_INFO("Configuring robot model: %s", robot_model_cfg.type.c_str());
    PluginLoader::loadPlugin("libwbc-robot_models-" + robot_model_cfg.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(robot_model_cfg.type));
    if(!robot_model->configure(robot_model_cfg)){
        ROS_ERROR("Failed to configure robot model");
        abort();
    }

    checkParam("qp_solver");
    std::string qp_solver;
    ros::param::get("qp_solver", qp_solver);

    ROS_INFO("Configuring solver: %s", "qpoases");
    PluginLoader::loadPlugin("libwbc-solvers-" + qp_solver + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(qp_solver));

    checkParam("wbc_config");
    xml_rpc_val.clear();
    ros::param::get("wbc_config", xml_rpc_val);
    fromROS(xml_rpc_val, wbc_config);

    ROS_INFO("Configuring scene: %s", "VelocitySceneQuadraticCost");
    scene = make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    if(!scene->configure(wbc_config)){
        ROS_ERROR("Failed to configure scene");
        abort();
    }

    ROS_INFO("Creating interfaces");

    // Input joint state
    joint_state.resize(robot_model->noOfJoints());
    joint_state.names = robot_model->jointNames();
    sub_feedback = nh->subscribe("joint_states", 1, &WbcNode::jointStateCallback, this);

    // Input floating base state
    if(robot_model_cfg.floating_base)
        sub_floating_base = nh->subscribe("floating_base_state", 1, &WbcNode::floatingBaseStateCallback, this);

    // Input references, task weights and activations
    for(auto w : wbc_config){
        ros::Subscriber sub;
        if(w.type == cart)
            sub = nh->subscribe<wbc_msgs::RigidBodyState>("ref_" + w.name, 1, boost::bind(&WbcNode::cartReferenceCallback, this, _1, w.name));
        else
            sub = nh->subscribe<trajectory_msgs::JointTrajectory>("ref_" + w.name, 1, boost::bind(&WbcNode::jntReferenceCallback, this, _1, w.name));
        subscribers.push_back(sub);
        sub = nh->subscribe<std_msgs::Float64MultiArray>("weights_" + w.name, 1, boost::bind(&WbcNode::taskWeightsCallback, this, _1, w.name));
        subscribers.push_back(sub);
        sub = nh->subscribe<std_msgs::Float64>("activation_" + w.name, 1, boost::bind(&WbcNode::taskActivationCallback, this, _1, w.name));
        subscribers.push_back(sub);
    }

    // Output task states
    for(auto w : wbc_config){
        ros::Publisher pub;
        if(w.type == cart){
            pub = nh->advertise<wbc_msgs::RigidBodyState>("status_" + w.name, 1);
        }
        else{
            pub = nh->advertise<sensor_msgs::JointState>("status_" + w.name, 1);
        }
        publishers_task_status.push_back(pub);
    }

    // Output task info
    for(auto w : wbc_config){
        ros::Publisher pub = nh->advertise<wbc_msgs::TaskStatus>("task_" + w.name, 1);
        publishers_task_info.push_back(pub);
    }

    // Input joint weights
    sub_joint_weights = nh->subscribe("joint_weights", 1, &WbcNode::jointWeightsCallback, this);

    // Solver output
    solver_output_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("solver_output", 1);

    // Stats on computation time
    pub_timing_stats = nh->advertise<wbc_msgs::WbcTimingStats>("timing_stats", 1);

    // WBC will send zeros if no setpoint is given from any controller
    has_setpoint = true;
}

WbcNode::~WbcNode(){
}

void WbcNode::jointStateCallback(const sensor_msgs::JointState& msg){
    fromROS(msg,joint_state);
    if(robot_model->hasFloatingBase()){
        if(has_floating_base_state)
            has_feedback = true;
    }
    else
        has_feedback = true;
}

void WbcNode::cartReferenceCallback(const ros::MessageEvent<wbc_msgs::RigidBodyState>& event, const std::string& constraint_name){
    fromROS(*event.getMessage(), reference_cart);
    scene->setReference(constraint_name, reference_cart);
}

void WbcNode::jntReferenceCallback(const ros::MessageEvent<trajectory_msgs::JointTrajectory>& event, const std::string& constraint_name){
    fromROS(*event.getMessage(), reference_jnt);
    scene->setReference(constraint_name, reference_jnt);
}

void WbcNode::taskActivationCallback(const ros::MessageEvent<std_msgs::Float64>& event, const std::string& constraint_name){
    scene->setTaskActivation(constraint_name, event.getMessage()->data);
}

void WbcNode::taskWeightsCallback(const ros::MessageEvent<std_msgs::Float64MultiArray>& event, const std::string& constraint_name){
    fromROS(*event.getMessage(), task_weights);
    scene->setTaskWeights(constraint_name, task_weights);
}

void WbcNode::jointWeightsCallback(const std_msgs::Float64MultiArray& msg){
    fromROS(msg, robot_model->jointNames(), joint_weights);
    scene->setJointWeights(joint_weights);
}

void WbcNode::floatingBaseStateCallback(const wbc_msgs::RigidBodyState& msg){
    fromROS(msg, floating_base_state);
    has_floating_base_state = true;
}

void WbcNode::updateController(){
    timing_stats.desired_period = 1.0 / control_rate;
    timing_stats.actual_period = (ros::Time::now() - stamp).toSec();
    stamp = ros::Time::now();

    ros::Time start = ros::Time::now();
    robot_model->update(joint_state, floating_base_state);
    timing_stats.time_robot_model_update = (ros::Time::now() - start).toSec();

    start = ros::Time::now();
    qp = scene->update();
    timing_stats.time_scene_update = (ros::Time::now() - start).toSec();

    start = ros::Time::now();
    solver_output = scene->solve(qp);
    timing_stats.time_solve = (ros::Time::now() - start).toSec();

    tasks_status = scene->updateTasksStatus();
    if(integrate)
        joint_integrator.integrate(robot_model->jointState(robot_model->actuatedJointNames()), solver_output, 1.0/control_rate);
    toROS(solver_output, solver_output_ros);
    solver_output_publisher.publish(solver_output_ros);

    publishTaskStatus();
    publishTaskInfo();

    timing_stats.time_per_cycle = (ros::Time::now() - stamp).toSec();
    timing_stats.header.stamp = ros::Time::now();
    pub_timing_stats.publish(timing_stats);
}

void WbcNode::publishTaskStatus(){
    for(int i = 0; i < wbc_config.size(); i++){
        const TaskConfig& w = wbc_config[i];
        if(w.type == cart){
            toROS(robot_model->rigidBodyState(w.ref_frame, w.tip), status_cart);
            publishers_task_status[i].publish(status_cart);
        }
        else{
            toROS(robot_model->jointState(w.joint_names), status_jnt);
            publishers_task_status[i].publish(status_jnt);
        }
    }
}

void WbcNode::publishTaskInfo(){
    task_status_msgs.resize(tasks_status.size());
    for(int i = 0; i < tasks_status.size(); i++){
        const TaskStatus& w = tasks_status[i];
        toROS(w, task_status_msgs[i]);
        publishers_task_info[i].publish(task_status_msgs[i]);
    }
}

int main(int argc, char** argv){
    WbcNode node(argc, argv);
    node.run();
}
