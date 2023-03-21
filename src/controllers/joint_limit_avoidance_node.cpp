#include "joint_limit_avoidance_node.hpp"
#include "../conversions.hpp"

using namespace ctrl_lib;
using namespace std;
using namespace rclcpp;

JointLimitAvoidanceNode::JointLimitAvoidanceNode(const string& node_name) : ControllerNode(node_name){

    // ROS2 does not support lists of complex data types as parameters, so we have to refer to this ugly hack here
    declare_parameter("joint_limits.names", vector<string>());
    base::JointLimits joint_limits;
    joint_limits.names = get_parameter("joint_limits.names").as_string_array();
    for(auto n : joint_limits.names){
        base::JointLimitRange range;
        string jn = "joint_limits." + n;
        declare_parameter(jn + ".position.max", 0.0);
        declare_parameter(jn + ".position.min", 0.0);
        range.max.position = get_parameter(jn + ".position.max").as_double();
        range.min.position = get_parameter(jn + ".position.min").as_double();
        joint_limits.elements.push_back(range);
    }

    declare_parameter("influence_distance", vector<double>());
    declare_parameter("p_gain", vector<double>());
    declare_parameter("max_control_output", vector<double>());

    vector<double> influence_distance = get_parameter("influence_distance").as_double_array();
    vector<double> p_gain             = get_parameter("p_gain").as_double_array();
    vector<double> max_control_output = get_parameter("max_control_output").as_double_array();

    controller = new JointLimitAvoidanceController(joint_limits, Eigen::Map<Eigen::VectorXd>(influence_distance.data(), influence_distance.size()));
    controller->setPGain(Eigen::Map<Eigen::VectorXd>(p_gain.data(),p_gain.size()));
    controller->setMaxControlOutput(Eigen::Map<Eigen::VectorXd>(max_control_output.data(),max_control_output.size()));

    // controller feedback
    sub_feedback = create_subscription<sensor_msgs::msg::JointState>("feedback", 1, bind(&JointLimitAvoidanceNode::feedbackCallback, this, placeholders::_1));
    // Ctrl output
    control_output_publisher = create_publisher<trajectory_msgs::msg::JointTrajectory>("control_output", 1);

    has_setpoint = true; // Controller does not need a setpoint
}

JointLimitAvoidanceNode::~JointLimitAvoidanceNode(){
    delete controller;
}

void JointLimitAvoidanceNode::feedbackCallback(const sensor_msgs::msg::JointState& msg){
    has_feedback = true;
    fromROS(msg, feedback);
}

void JointLimitAvoidanceNode::updateController(){
    control_output = controller->update(feedback);
    toROS(control_output, control_output_msg);
    control_output_publisher->publish(control_output_msg);
}

int main(int argc, char** argv){
    init(argc, argv);
    spin(make_shared<JointLimitAvoidanceNode>("joint_limit_avoidance"));
    return 0;
}
