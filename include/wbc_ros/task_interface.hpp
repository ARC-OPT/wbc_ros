#ifndef WBC_ROS_TASK_INTERFACE_CPP
#define WBC_ROS_TASK_INTERFACE_CPP

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <wbc/core/Task.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc_ros/conversions.hpp>

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
         wbc::RobotModelPtr robot_model;
         std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;

         TaskInterface(wbc::TaskPtr task, 
                       wbc::RobotModelPtr robot_model, 
                       std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
         void updateWeights();
         void task_weight_callback(const TaskWeightMsgPtr msg);
         void task_activation_callback(const TaskActivationMsgPtr msg);
         virtual void updateReference() = 0;
         virtual void publishStatus() = 0;
         virtual void reset() = 0;
    };

    using TaskInterfacePtr = std::shared_ptr<TaskInterface>;    

    class SpatialVelocityTaskInterface : public TaskInterface{
      protected:
         using ReferenceMsg = wbc_msgs::msg::RigidBodyState;
         using ReferenceMsgPtr = std::shared_ptr<ReferenceMsg>;
         using ReferenceSubscription = rclcpp::Subscription<ReferenceMsg>::SharedPtr;
         using RTReferenceBuffer = realtime_tools::RealtimeBuffer<ReferenceMsgPtr>;

         using StatusMsg = wbc_msgs::msg::RigidBodyState;
         using StatusPublisher = rclcpp::Publisher<StatusMsg>;
         using RTStatusPublisher = realtime_tools::RealtimePublisher<StatusMsg>;


      public:
         ReferenceSubscription reference_subscriber;
         StatusPublisher::SharedPtr status_publisher;
         std::unique_ptr<RTStatusPublisher> rt_status_publisher;
         RTReferenceBuffer rt_reference_buffer;
         ReferenceMsgPtr reference_msg;

         wbc::types::RigidBodyState status;
         wbc::types::RigidBodyState reference;

         SpatialVelocityTaskInterface(wbc::TaskPtr task, 
                                      wbc::RobotModelPtr robot_model, 
                                      std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
         virtual void updateReference();
         virtual void publishStatus();
         virtual void reset();
         void reference_callback(const ReferenceMsgPtr msg);
    };

    /*class SpatialAccelerationTaskInterface : public TaskInterface{
      public:
        wbc::types::Pose status_pose;
        wbc::types::Twist status_twist;
        wbc::types::SpatialAcceleration reference;         

        SpatialAccelerationTaskInterface(wbc::TaskPtr task, 
                                         wbc::RobotModelPtr robot_model, 
                                         std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
        virtual void updateReference();
        virtual void updateStatus();      
    };

    class CoMVelocityTaskInterface : public TaskInterface{
      public:
        CoMVelocityTaskInterface(wbc::TaskPtr task, 
                                 wbc::RobotModelPtr robot_model, 
                                 std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
        virtual void updateReference();
        virtual void updateStatus();
    };

    class CoMAccelerationTaskInterface : public TaskInterface{
      public:
        CoMAccelerationTaskInterface(wbc::TaskPtr task, 
                                     wbc::RobotModelPtr robot_model, 
                                     std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
        virtual void updateReference();
        virtual void updateStatus();
    };

    class JointVelocityTaskInterface : public TaskInterface{
      public:
         JointVelocityTaskInterface(wbc::TaskPtr task, 
                                    wbc::RobotModelPtr robot_model, 
                                    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
        virtual void updateReference();
        virtual void updateStatus();
    };

    class JointAccelerationTaskInterface : public TaskInterface{
      public:
         JointAccelerationTaskInterface(wbc::TaskPtr task, 
                                        wbc::RobotModelPtr robot_model, 
                                        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
        virtual void updateReference();
        virtual void updateStatus();
    };

    class ContactForceTaskInterface : public TaskInterface{
      public:
         wbc::types::Wrench reference;       

         ContactForceTaskInterface(wbc::TaskPtr task, 
                                   wbc::RobotModelPtr robot_model, 
                                   std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
        virtual void updateReference();
        virtual void updateStatus();
    };*/

} // namespace wbc_ros


#endif