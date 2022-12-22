#ifndef CARTESIAN_POSITION_CONTROLLER_NODE_HPP
#define CARTESIAN_POSITION_CONTROLLER_NODE_HPP

#include <ros/ros.h>

#include <wbc/controllers/CartesianPosPDController.hpp>

using namespace ctrl_lib;

class CartesianPositionControllerNode{
protected:
   ros::NodeHandle* nh;
   std::string state;
   double control_rate;
   CartesianPosPDController controller;

public:
    CartesianPositionControllerNode(int argc, char** argv);
    ~CartesianPositionControllerNode();

    void update();
    void run();
};

#endif
