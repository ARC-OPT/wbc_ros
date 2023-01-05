#ifndef WBC_ROS_NODE_HPP
#define WBC_ROS_NODE_HPP

#include <ros/ros.h>

#include <wbc_msgs/RigidBodyState.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <wbc/core/RobotModel.hpp>
#include <wbc/core/PluginLoader.hpp>
#include <wbc/core/Scene.hpp>
#include <wbc/core/QPSolver.hpp>
#include <wbc/tools/JointIntegrator.hpp>

#include <base/commands/Joints.hpp>

#include "conversions.hpp"

class WbcNode{
   enum controllerState{PRE_OPERATIONAL = 0,
                        NO_FLOATING_BASE_STATE,
                        NO_JOINT_STATE,
                        RUNNING};

    std_msgs::String controllerStateToStringMsg(controllerState s){
        std_msgs::String msg;
        switch(s){
            case PRE_OPERATIONAL: msg.data = "PRE_OPERATIONAL";
            case NO_FLOATING_BASE_STATE: msg.data = "NO_FLOATING_BASE_STATE";
            case NO_JOINT_STATE: msg.data = "NO_JOINT_STATE";
            case RUNNING: msg.data = "RUNNING";
        }
        return msg;
    }
protected:
   ros::NodeHandle* nh;
   wbc::WbcScenePtr scene;
   wbc::RobotModelPtr robot_model;
   wbc::QPSolverPtr solver;

   base::samples::Joints joint_state;
   base::samples::RigidBodyStateSE3 reference_cart;
   base::samples::RigidBodyStateSE3 floating_base_state;
   base::commands::Joints reference_jnt;
   base::VectorXd task_weights;
   wbc::JointWeights joint_weights;
   bool integrate;
   double control_rate;
   controllerState state;
   wbc::HierarchicalQP qp;
   base::commands::Joints solver_output;
   wbc::JointIntegrator joint_integrator;
   std::vector<wbc::TaskConfig> wbc_config;
   bool has_joint_state;
   bool has_floating_base_state;
   wbc::TasksStatus tasks_status;
   std::vector<wbc_msgs::TaskStatus> task_status_msgs;

   trajectory_msgs::JointTrajectory solver_output_ros;

   std::vector<ros::Subscriber> subscribers;
   std::vector<ros::Publisher> publishers_task_status;
   std::vector<ros::Publisher> publishers_task_info;
   ros::Subscriber sub_joint_state;
   ros::Subscriber sub_joint_weights;
   ros::Subscriber sub_floating_base;
   ros::Publisher solver_output_publisher;
   ros::Publisher state_publisher;
   wbc_msgs::RigidBodyState status_cart;
   sensor_msgs::JointState status_jnt;

   void jointStateCallback(const sensor_msgs::JointState& msg);
   void cartReferenceCallback(const ros::MessageEvent<wbc_msgs::RigidBodyState>& event, const std::string& constraint_name);
   void jntReferenceCallback(const ros::MessageEvent<trajectory_msgs::JointTrajectory>& event, const std::string& constraint_name);
   void taskActivationCallback(const ros::MessageEvent<std_msgs::Float64>& event, const std::string& constraint_name);
   void taskWeightsCallback(const ros::MessageEvent<std_msgs::Float64MultiArray>& event, const std::string& constraint_name);
   void jointWeightsCallback(const std_msgs::Float64MultiArray& msg);
   void floatingBaseStateCallback(const wbc_msgs::RigidBodyState& msg);

public:
   WbcNode(int argc, char** argv);
   ~WbcNode();
   void update();
   void publishTaskStatus();
   void publishTaskInfo();
   void run();
};

#endif
