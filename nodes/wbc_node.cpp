#include "wbc_node.hpp"
#include <wbc/scenes/VelocitySceneQuadraticCost.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include "conversions.hpp"

using namespace std;

namespace wbc{

WbcNode::WbcNode(int argc, char** argv){

    ros::init(argc, argv, "wbc");
    nh = new ros::NodeHandle();

    XmlRpc::XmlRpcValue xml_rpc_val;
    ros::param::get("robot_model_config", xml_rpc_val);
    if(!xml_rpc_val.valid()){
        ROS_ERROR("ROS parameter 'robot_model_config' has not been set");
        abort();
    }
    RobotModelConfig robot_model_cfg;
    fromROS(xml_rpc_val, robot_model_cfg);

    ROS_INFO("Configuring robot model: %s", robot_model_cfg.type.c_str());

    PluginLoader::loadPlugin("libwbc-robot_models-" + robot_model_cfg.type + ".so");
    robot_model = shared_ptr<RobotModel>(RobotModelFactory::createInstance(robot_model_cfg.type));
    if(!robot_model->configure(robot_model_cfg)){
        ROS_ERROR("Failed to configure robot model");
        abort();
    }

    xml_rpc_val.clear();
    ros::param::get("wbc_config", xml_rpc_val);
    if(!xml_rpc_val.valid()){
        ROS_ERROR("ROS parameter 'wbc_config' has not been set");
        abort();
    }
    vector<ConstraintConfig> wbc_config;
    fromROS(xml_rpc_val, wbc_config);

    ROS_INFO("Configuring solver: %s", "qpoases");
    solver = make_shared<QPOASESSolver>();

    ROS_INFO("Configuring scene: %s", "VelocitySceneQuadraticCost");
    scene = make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    if(!scene->configure(wbc_config)){
        ROS_ERROR("Failed to configure scene");
        abort();
    }

    ROS_INFO("Creating interfaces");

    // Input joint state
    is_initialized = false;
    joint_state.resize(robot_model->noOfJoints());
    joint_state.names = robot_model->jointNames();
    ros::Subscriber sub = nh->subscribe("joint_states", 1, &WbcNode::jointStateCallback, this);
    subscribers.push_back(sub);

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
    sub = nh->subscribe("joint_weights", 1, &WbcNode::jointWeightsCallback, this);

    // Solver output
    solver_output_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("solver_output", 1);
}

WbcNode::~WbcNode(){
    delete nh;
}

void WbcNode::jointStateCallback(const sensor_msgs::JointState& msg){
    fromROS(msg,joint_state);
    for(uint i = 0; i < joint_state.size(); i++){
        if(!joint_state[i].hasPosition()) // TODO: Also check velocity?
            return;
    }
    robot_model->update(joint_state); // Only update if all joints have a valid joint state
    is_initialized = true;
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
    qp = scene->update();
    solver_output = scene->solve(qp);
    toROS(solver_output, solver_output_ros);
    solver_output_publisher.publish(solver_output_ros);
}

}

int main(int argc, char** argv)
{
    wbc::WbcNode node(argc, argv);
    /*ros::Subscriber sub;
    for(auto w : wbc_config)
       sub = nh.subscribe("ref_" + w.name, 1, callback);*/

    double rate;
    ros::param::get("rate", rate);
    ros::Rate loop_rate(rate);
    ROS_INFO("Whole-Body Controller is running");
    while(ros::ok()){
        if(node.isInitialized())
            node.solve();
        else
            ROS_WARN_DELAYED_THROTTLE(5,"WBC did not receive a valid joint state for all configured joints");
        loop_rate.sleep();
    }
    return 0;
}
