#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
@brief Base class for all controllers including wbc_node. Will peridically call updateController() if feedback
and setpoint are available.
*/
class ControllerNode : public rclcpp::Node{
    enum controllerState{
        PRE_OPERATIONAL = 0,
        NO_FEEDBACK,
        NO_SETPOINT,
        RUNNING
    };
    std_msgs::msg::String controllerState2StringMsg(controllerState s){
        std_msgs::msg::String msg;
        if(s == PRE_OPERATIONAL)
            msg.data = "PRE_OPERATIONAL";
        else if(s == NO_FEEDBACK)
            msg.data = "NO_FEEDBACK";
        else if(s == NO_SETPOINT)
            msg.data = "NO_SETPOINT";
        else if(s == RUNNING)
            msg.data = "RUNNING";
        return msg;
    }


protected:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher;
    rclcpp::TimerBase::SharedPtr timer_update;
    rclcpp::TimerBase::SharedPtr timer_state_pub;
    controllerState state;
    std::string node_name;
    double control_rate;
    bool has_feedback;
    bool has_setpoint;

    virtual void updateController() = 0;
    void publishState();
public:
    ControllerNode(std::string node_name);
    virtual ~ControllerNode(){}
    void update();
};

#endif
