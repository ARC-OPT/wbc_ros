#include "cartesian_position_controller_node.hpp"

using namespace ctrl_lib;

CartesianPositionControllerNode::CartesianPositionControllerNode(int argc, char** argv) : state("NO_FEEDBACK"){
    ros::init(argc, argv, "cartesian_position_controller");
    nh = new ros::NodeHandle();

    if(!ros::param::has("control_rate")){
        ROS_ERROR("WBC parameter control_rate has not been set");
        abort();
    }
    ros::param::get("control_rate", control_rate);
}

CartesianPositionControllerNode::~CartesianPositionControllerNode(){

}

void CartesianPositionControllerNode::update(){

}

void CartesianPositionControllerNode::run(){
    ros::Rate loop_rate(control_rate);
    ROS_INFO("Whole-Body Controller is running");
    while(ros::ok()){
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    CartesianPositionControllerNode node(argc, argv);
    node.run();
    return 0;
}
