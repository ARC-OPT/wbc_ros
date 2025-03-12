#ifndef WBC_ROS_TASK_INTERFACE_CPP
#define WBC_ROS_TASK_INTERFACE_CPP

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64.hpp>
#include <wbc/core/Task.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc_ros/conversions.hpp>
#include <wbc/controllers/CartesianPosPDController.hpp>
#include <wbc/controllers/JointPosPDController.hpp>

namespace wbc_ros{
    
class TaskInterface{
      protected:
         using TaskWeightMsg = std_msgs::msg::Float64MultiArray;
         using TaskWeightMsgPtr = std::shared_ptr<TaskWeightMsg>;
         using TaskWeightSubscription = rclcpp::Subscription<TaskWeightMsg>::SharedPtr;
         using RTTaskWeightBuffer = realtime_tools::RealtimeBuffer<TaskWeightMsgPtr>;

         using TaskActivationMsg = std_msgs::msg::Float64;
         using TaskActivationMsgPtr = std::shared_ptr<TaskActivationMsg>;
         using TaskActivationSubscription = rclcpp::Subscription<TaskActivationMsg>::SharedPtr;
         using RTTaskActivationBuffer = realtime_tools::RealtimeBuffer<TaskActivationMsgPtr>;

         TaskWeightSubscription task_weight_subscriber;
         RTTaskWeightBuffer rt_task_weight_buffer;
         TaskWeightMsgPtr task_weight_msg;

         TaskActivationSubscription task_activation_subscriber;
         RTTaskActivationBuffer rt_task_activation_buffer;
         TaskActivationMsgPtr task_activation_msg;

      public:
         wbc::TaskPtr task;
         std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;

         TaskInterface(wbc::TaskPtr task, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
         void updateTaskWeights();
         void task_weight_callback(const TaskWeightMsgPtr msg);
         void task_activation_callback(const TaskActivationMsgPtr msg);
         virtual void updateTask() = 0;
    };
    using TaskInterfacePtr = std::shared_ptr<TaskInterface>;    

    class SpatialVelocityTaskInterface : public TaskInterface{
      protected:
         using SetpointMsg = robot_control_msgs::msg::RigidBodyState;
         using SetpointMsgPtr = std::shared_ptr<SetpointMsg>;
         using SetpointSubscription = rclcpp::Subscription<SetpointMsg>::SharedPtr;
         using RTSetpointBuffer = realtime_tools::RealtimeBuffer<SetpointMsgPtr>;

      public:
         std::shared_ptr<wbc::CartesianPosPDController> controller;
         wbc::RobotModelPtr robot_model;
         SetpointSubscription setpoint_subscriber;
         RTSetpointBuffer rt_setpoint_buffer;
         SetpointMsgPtr setpoint_msg;

         wbc::types::Pose pose;
         wbc::types::RigidBodyState setpoint;
         wbc::types::Twist control_output;

         SpatialVelocityTaskInterface(wbc::TaskPtr task, 
                                      wbc::CartesianPosPDControllerPtr controller,
                                      wbc::RobotModelPtr robot_model, 
                                      std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
         virtual void updateTask();
         void setpoint_callback(const SetpointMsgPtr msg);
    };
    using SpatialVelocityTaskInterfacePtr = std::shared_ptr<SpatialVelocityTaskInterface>;    

    class SpatialAccelerationTaskInterface : public TaskInterface{
      protected:
         using SetpointMsg = robot_control_msgs::msg::RigidBodyState;
         using SetpointMsgPtr = std::shared_ptr<SetpointMsg>;
         using SetpointSubscription = rclcpp::Subscription<SetpointMsg>::SharedPtr;
         using RTSetpointBuffer = realtime_tools::RealtimeBuffer<SetpointMsgPtr>;

      public:
         wbc::CartesianPosPDControllerPtr controller;
         wbc::RobotModelPtr robot_model;
         SetpointSubscription setpoint_subscriber;
         RTSetpointBuffer rt_setpoint_buffer;
         SetpointMsgPtr setpoint_msg;

         wbc::types::Pose pose;
         wbc::types::Twist twist;
         wbc::types::RigidBodyState setpoint;
         wbc::types::SpatialAcceleration control_output;

         SpatialAccelerationTaskInterface(wbc::TaskPtr task, 
                                          wbc::CartesianPosPDControllerPtr controller,
                                          wbc::RobotModelPtr robot_model, 
                                          std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
         virtual void updateTask();
         void setpoint_callback(const SetpointMsgPtr msg);
    };

    class CoMVelocityTaskInterface : public TaskInterface{
      protected:
         using SetpointMsg = robot_control_msgs::msg::RigidBodyState;
         using SetpointMsgPtr = std::shared_ptr<SetpointMsg>;
         using SetpointSubscription = rclcpp::Subscription<SetpointMsg>::SharedPtr;
         using RTSetpointBuffer = realtime_tools::RealtimeBuffer<SetpointMsgPtr>;

      public:
         wbc::CartesianPosPDControllerPtr controller;
         wbc::RobotModelPtr robot_model;
         SetpointSubscription setpoint_subscriber;
         RTSetpointBuffer rt_setpoint_buffer;
         SetpointMsgPtr setpoint_msg;

         wbc::types::Pose pose;
         wbc::types::RigidBodyState setpoint;
         wbc::types::Twist control_output;
               
        CoMVelocityTaskInterface(wbc::TaskPtr task, 
                                 wbc::CartesianPosPDControllerPtr controller,
                                 wbc::RobotModelPtr robot_model, 
                                 std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
        virtual void updateTask();
        void setpoint_callback(const SetpointMsgPtr msg);
    };

    class CoMAccelerationTaskInterface : public TaskInterface{
      protected:
         using SetpointMsg = robot_control_msgs::msg::RigidBodyState;
         using SetpointMsgPtr = std::shared_ptr<SetpointMsg>;
         using SetpointSubscription = rclcpp::Subscription<SetpointMsg>::SharedPtr;
         using RTSetpointBuffer = realtime_tools::RealtimeBuffer<SetpointMsgPtr>;

      public:
         wbc::CartesianPosPDControllerPtr controller;
         wbc::RobotModelPtr robot_model;
         SetpointSubscription setpoint_subscriber;
         RTSetpointBuffer rt_setpoint_buffer;
         SetpointMsgPtr setpoint_msg;

         wbc::types::Pose pose;
         wbc::types::Twist twist;
         wbc::types::RigidBodyState setpoint;
         wbc::types::SpatialAcceleration control_output;
               
         CoMAccelerationTaskInterface(wbc::TaskPtr task, 
                                      wbc::CartesianPosPDControllerPtr controller,
                                      wbc::RobotModelPtr robot_model, 
                                      std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
        virtual void updateTask();
        void setpoint_callback(const SetpointMsgPtr msg);
    };

    class JointVelocityTaskInterface : public TaskInterface{
      protected:
         using SetpointMsg = robot_control_msgs::msg::JointCommand;
         using SetpointMsgPtr = std::shared_ptr<SetpointMsg>;
         using SetpointSubscription = rclcpp::Subscription<SetpointMsg>::SharedPtr;
         using RTSetpointBuffer = realtime_tools::RealtimeBuffer<SetpointMsgPtr>;
      public:
         SetpointSubscription setpoint_subscriber;
         RTSetpointBuffer rt_setpoint_buffer;
         SetpointMsgPtr setpoint_msg;

         wbc::JointPosPDControllerPtr controller;
         wbc::RobotModelPtr robot_model;
         wbc::types::JointCommand setpoint;
         wbc::types::JointCommand joint_state;
         Eigen::VectorXd control_output;
         JointVelocityTaskInterface(wbc::TaskPtr task, 
                                    wbc::JointPosPDControllerPtr controller,
                                    wbc::RobotModelPtr robot_model, 
                                    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
         virtual void updateTask();
         void setpoint_callback(const SetpointMsgPtr msg);
    };

    class JointAccelerationTaskInterface : public TaskInterface{
      protected:
         using SetpointMsg = robot_control_msgs::msg::JointCommand;
         using SetpointMsgPtr = std::shared_ptr<SetpointMsg>;
         using SetpointSubscription = rclcpp::Subscription<SetpointMsg>::SharedPtr;
         using RTSetpointBuffer = realtime_tools::RealtimeBuffer<SetpointMsgPtr>;
      public:
         SetpointSubscription setpoint_subscriber;
         RTSetpointBuffer rt_setpoint_buffer;
         SetpointMsgPtr setpoint_msg;

         wbc::JointPosPDControllerPtr controller;
         wbc::RobotModelPtr robot_model;
         wbc::types::JointCommand setpoint;
         wbc::types::JointCommand joint_state;
         Eigen::VectorXd control_output;
         JointAccelerationTaskInterface(wbc::TaskPtr task, 
                                        wbc::JointPosPDControllerPtr controller,
                                        wbc::RobotModelPtr robot_model, 
                                        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
         virtual void updateTask();
         void setpoint_callback(const SetpointMsgPtr msg);
    };

    class ContactForceTaskInterface : public TaskInterface{
      protected:
         using SetpointMsg = geometry_msgs::msg::Wrench;
         using SetpointMsgPtr = std::shared_ptr<SetpointMsg>;
         using SetpointSubscription = rclcpp::Subscription<SetpointMsg>::SharedPtr;
         using RTSetpointBuffer = realtime_tools::RealtimeBuffer<SetpointMsgPtr>;                                       
      public:
         SetpointSubscription setpoint_subscriber;
         RTSetpointBuffer rt_setpoint_buffer;
         SetpointMsgPtr setpoint_msg;
         wbc::types::Wrench setpoint;
         ContactForceTaskInterface(wbc::TaskPtr task,
                                   std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
         virtual void updateTask();
         virtual void setpoint_callback(const SetpointMsgPtr msg                                                                                                                                                                                                                                                                                                                      );
    };

} // namespace wbc_ros


#endif