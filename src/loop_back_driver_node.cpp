#include "loop_back_driver_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std;
using namespace rclcpp;

namespace wbc_ros{

LoopBackDriverNode::LoopBackDriverNode(const rclcpp::NodeOptions& options) : Node("joints", options){
    RCLCPP_INFO(get_logger(), "Initializing Controller: %s", "joints");

    declare_parameter("control_rate", 1000.0);
    double control_rate = get_parameter("control_rate").get_parameter_value().get<double>();

    declare_parameter("initial_joint_state.name", vector<string>());
    declare_parameter("initial_joint_state.position", vector<double>());
    declare_parameter("initial_joint_state.velocity", vector<double>());
    declare_parameter("initial_joint_state.effort", vector<double>());
    joint_state.name     = get_parameter("initial_joint_state.name").as_string_array();
    joint_state.position = get_parameter("initial_joint_state.position").as_double_array();
    joint_state.velocity = get_parameter("initial_joint_state.velocity").as_double_array();
    joint_state.effort   = get_parameter("initial_joint_state.effort").as_double_array();

    declare_parameter("noise_std_dev", 1e-4);
    noise_std_dev = get_parameter("noise_std_dev").as_double();

    timer_update = create_wall_timer(std::chrono::duration<double>(1.0/control_rate), std::bind(&LoopBackDriverNode::update, this));

    sub_command     = create_subscription<trajectory_msgs::msg::JointTrajectory>("command", 1, bind(&LoopBackDriverNode::commandCallback, this, placeholders::_1));
    pub_joint_state = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
}

LoopBackDriverNode::~LoopBackDriverNode(){
}

void LoopBackDriverNode::commandCallback(const trajectory_msgs::msg::JointTrajectory& msg){
    for(uint i = 0; i < msg.joint_names.size(); i++){
        string n = msg.joint_names[i];
        auto it = find(joint_state.name.begin(), joint_state.name.end(), n);
        if(it == joint_state.name.end()){
            RCLCPP_ERROR(get_logger(), "Joint %s is in command but this joint has not been configured in initial joint state", n.c_str());
            abort();
        }
        int idx = it - joint_state.name.begin();
        joint_state.position[idx] = msg.points[0].positions[i] + whiteNoise(noise_std_dev);
        joint_state.velocity[idx] = msg.points[0].velocities[i] + whiteNoise(noise_std_dev);
        joint_state.effort[idx]   = msg.points[0].effort[i] + whiteNoise(noise_std_dev);
    }
}

double LoopBackDriverNode::whiteNoise(const double std_dev){
    double rand_no = ( rand() / ( (double)RAND_MAX ) );
    while( rand_no == 0 )
        rand_no = ( rand() / ( (double)RAND_MAX ) );

    double tmp = cos( ( 2.0 * (double)M_PI ) * rand() / ( (double)RAND_MAX ) );
    return std_dev * sqrt( -2.0 * log( rand_no ) ) * tmp;
}

void LoopBackDriverNode::update(){
    joint_state.header.stamp = get_clock()->now();
    pub_joint_state->publish(joint_state);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(wbc_ros::LoopBackDriverNode)
