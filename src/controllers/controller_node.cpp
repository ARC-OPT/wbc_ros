#include "controller_node.hpp"

using namespace rclcpp;
using namespace std;

ControllerNode::ControllerNode(string node_name) : Node(node_name), state(PRE_OPERATIONAL), has_feedback(false), has_setpoint(false){

    RCLCPP_INFO(this->get_logger(), "Initializing Controller: %s", node_name.c_str());

    this->declare_parameter("control_rate", 1000.0);
    control_rate = this->get_parameter("control_rate").get_parameter_value().get<double>();

    state_publisher = this->create_publisher<std_msgs::msg::String>("state", 1);
    timer_update = this->create_wall_timer(std::chrono::duration<double>(1.0/control_rate), std::bind(&ControllerNode::update, this));
    timer_state_pub = this->create_wall_timer(1000ms, std::bind(&ControllerNode::publishState, this));
}

void ControllerNode::publishState(){
    state_publisher->publish(controllerState2StringMsg(state));
}

void ControllerNode::update(){
    switch(state){
        case PRE_OPERATIONAL:{
            state = NO_FEEDBACK;
            break;
        }
        case NO_FEEDBACK:{
            if(has_feedback)
                state = NO_SETPOINT;
            else
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s: No feedback", node_name.c_str());
            break;
        }
        case NO_SETPOINT:{
            if(has_setpoint)
                state = RUNNING;
            else
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s: No setpoint", node_name.c_str());
            break;
        }
        case RUNNING:{
            updateController();
            break;
        }
        default:{
            RCLCPP_ERROR(this->get_logger(), "Invalid controller state: %i", state);
            abort();
        }
    }
}
