#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

class ControllerNode{
    enum controllerState{
        PRE_OPERATIONAL = 0,
        NO_FEEDBACK,
        NO_SETPOINT,
        RUNNING
    };
    std_msgs::String controllerState2StringMsg(controllerState s){
        std_msgs::String msg;
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
    ros::NodeHandle* nh;
    ros::Subscriber sub_feedback;
    ros::Subscriber sub_setpoint;
    ros::Publisher control_output_publisher;
    ros::Publisher state_publisher;
    ros::Time stamp;

    controllerState state;
    std::string node_name;
    double control_rate;
    bool has_feedback;
    bool has_setpoint;

    virtual void updateController() = 0;
    void checkParam(std::string name);
public:
    ControllerNode(int argc, char** argv);
    virtual ~ControllerNode();
    void run();
};

#endif
