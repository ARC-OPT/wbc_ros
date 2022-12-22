#include "wbc_node.hpp"
#include <wbc/scenes/VelocitySceneQuadraticCost.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/core/RobotModelFactory.hpp>
#include <wbc/core/QPSolverFactory.hpp>
#include "conversions.hpp"
#include <std_msgs/String.h>

using namespace std;
using namespace wbc;

WbcNode::WbcNode(int argc, char** argv) : state("NO_JOINT_STATE"){

    ros::init(argc, argv, "wbc");
    nh = new ros::NodeHandle();

    if(!ros::param::has("control_rate")){
        ROS_ERROR("WBC parameter control_rate has not been set");
        abort();
    }
    ros::param::get("control_rate", control_rate);

    if(!ros::param::has("integrate")){
        ROS_ERROR("WBC parameter integrate has not been set");
        abort();
    }
    ros::param::get("integrate", integrate);

    if(!ros::param::has("robot_model_config")){
        ROS_ERROR("ROS parameter 'robot_model_config' has not been set");
        abort();
    }
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

    if(!ros::param::has("qp_solver")){
        ROS_ERROR("ROS parameter 'qp_solver' has not been set");
        abort();
    }
    std::string qp_solver;
    ros::param::get("qp_solver", qp_solver);

    ROS_INFO("Configuring solver: %s", "qpoases");
    PluginLoader::loadPlugin("libwbc-solvers-" + qp_solver + ".so");
    solver = shared_ptr<QPSolver>(QPSolverFactory::createInstance(qp_solver));

    if(!ros::param::has("wbc_config")){
        ROS_ERROR("ROS parameter 'wbc_config' has not been set");
        abort();
    }
    xml_rpc_val.clear();
    ros::param::get("wbc_config", xml_rpc_val);
    vector<TaskConfig> wbc_config;
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
    sub_joint_state = nh->subscribe("joint_states", 1, &WbcNode::jointStateCallback, this);

    // Input floating base state
    if(robot_model_cfg.floating_base == true){
        ROS_ERROR("Floating base is not yet supported");
        abort();
        //ros::Subscriber sub = nh->subscribe("floating_base_state", 1, &WbcNode::floatingBaseStateCallback, this));
        //subscribers.push_back(sub);
    }

    // Input references, task weights and activations
    for(auto w : wbc_config){
        ros::Subscriber sub;
        if(w.type == cart){
            sub = nh->subscribe<geometry_msgs::TwistStamped>("ref_" + w.name, 1, boost::bind(&WbcNode::cartReferenceCallback, this, _1, w.name));
            subscribers.push_back(sub);
        }
        else{
            sub = nh->subscribe<trajectory_msgs::JointTrajectory>("ref_" + w.name, 1, boost::bind(&WbcNode::jntReferenceCallback, this, _1, w.name));
            subscribers.push_back(sub);
        }
        sub = nh->subscribe<std_msgs::Float64MultiArray>("weights_" + w.name, 1, boost::bind(&WbcNode::taskWeightsCallback, this, _1, w.name));
        subscribers.push_back(sub);
        sub = nh->subscribe<std_msgs::Float64>("activation_" + w.name, 1, boost::bind(&WbcNode::taskActivationCallback, this, _1, w.name));
        subscribers.push_back(sub);
    }

    // Input joint weights
    ros::Subscriber sub = nh->subscribe("joint_weights", 1, &WbcNode::jointWeightsCallback, this);

    // Solver output
    solver_output_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("solver_output", 1);

    // State
    state_publisher = nh->advertise<std_msgs::String>("state", 1);
}

WbcNode::~WbcNode(){
    delete nh;
}

void WbcNode::jointStateCallback(const sensor_msgs::JointState& msg){
    fromROS(msg,joint_state);
    robot_model->update(joint_state);
    state = "RUNNING";
}

void WbcNode::cartReferenceCallback(const ros::MessageEvent<geometry_msgs::TwistStamped>& event, const std::string& constraint_name){
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

void WbcNode::solve(){
    std_msgs::String msg;
    msg.data = state;
    state_publisher.publish(std_msgs::String(msg));

    if(state == "NO_JOINT_STATE"){
        ROS_WARN_DELAYED_THROTTLE(5,"WBC did not receive a valid joint state for all configured joints");
        return;
    }
    qp = scene->update();
    solver_output = scene->solve(qp);
    if(integrate)
        joint_integrator.integrate(robot_model->jointState(robot_model->actuatedJointNames()), solver_output, 1.0/control_rate);
    toROS(solver_output, solver_output_ros);
    solver_output_publisher.publish(solver_output_ros);
}

void WbcNode::run(){
    ros::Rate loop_rate(control_rate);
    ROS_INFO("Whole-Body Controller is running");
    while(ros::ok()){
        solve();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    WbcNode node(argc, argv);
    node.run();
}
