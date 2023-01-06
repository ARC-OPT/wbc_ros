#include "controller_node.hpp"

ControllerNode::ControllerNode(int argc, char** argv) : state(PRE_OPERATIONAL), has_feedback(false),has_setpoint(false){
    ros::init(argc, argv, "controller");
    nh = new ros::NodeHandle();
    node_name = ros::this_node::getName();

    ROS_INFO("Initializing Controller: %s", node_name.c_str());

    checkParam("control_rate");
    ros::param::get("control_rate", control_rate);

    state_publisher = nh->advertise<std_msgs::String>("state", 1);
}

ControllerNode::~ControllerNode(){
    delete nh;
}

void ControllerNode::checkParam(std::string name){
    if(!ros::param::has(name)){
        ROS_ERROR("%s: Parameter %s has not been set", node_name.c_str(), name.c_str());
        abort();
    }
}

void ControllerNode::run(){
    ros::Rate loop_rate(control_rate);
    ROS_INFO("Running: %s", node_name.c_str());
    while(ros::ok()){

        switch(state){
            case PRE_OPERATIONAL:{
                state = NO_FEEDBACK;
                break;
            }
            case NO_FEEDBACK:{
                if(has_feedback)
                    state = NO_SETPOINT;
                else
                    ROS_WARN_DELAYED_THROTTLE(5, "%s: No feedback", node_name.c_str());
                break;
            }
            case NO_SETPOINT:{
                if(has_setpoint)
                    state = RUNNING;
                else
                    ROS_DEBUG_DELAYED_THROTTLE(5, "%s: No setpoint", node_name.c_str());
                break;
            }
            case RUNNING:{
                updateController();
                break;
            }
            default:{
                ROS_ERROR("Invalid controller state: %i", state);
                abort();
            }
        }

        if((ros::Time::now() - stamp).toSec() > 1){
            state_publisher.publish(controllerState2StringMsg(state));
            stamp = ros::Time::now();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
