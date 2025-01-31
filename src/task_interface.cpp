#include <wbc_ros/task_interface.hpp>
#include <wbc/tasks/SpatialVelocityTask.hpp>
#include <wbc/tasks/SpatialAccelerationTask.hpp>
#include <wbc/tasks/CoMVelocityTask.hpp>
#include <wbc/tasks/CoMAccelerationTask.hpp>
#include <wbc/tasks/JointVelocityTask.hpp>
#include <wbc/tasks/JointAccelerationTask.hpp>
#include <wbc/tasks/ContactForceTask.hpp>

using namespace wbc;
using namespace std;
using namespace rclcpp;

namespace wbc_ros{
    TaskInterface::TaskInterface(TaskPtr task, 
                                 RobotModelPtr robot_model, 
                                 shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
            task(task), 
            robot_model(robot_model), 
            node(node){  
        task_weight_subscriber = node->create_subscription<TaskWeightMsg>("~/" + task->config.name + "/task_weights", SystemDefaultsQoS(), bind(&TaskInterface::task_weight_callback, this, placeholders::_1));
        task_activation_subscriber = node->create_subscription<TaskActivationMsg>("~/" + task->config.name + "/activation", SystemDefaultsQoS(), bind(&TaskInterface::task_activation_callback, this, placeholders::_1));
    }

    void TaskInterface::updateWeights(){
        // update task weights
        task_weight_msg = *rt_task_weight_buffer.readFromRT();
        if(task_weight_msg.get())
            task->setWeights(Eigen::Map<Eigen::VectorXd>(task_weight_msg->data.data(), task_weight_msg->data.size()));
            
        // Update task activations
        task_activation_msg = *rt_task_activation_buffer.readFromRT();
        if (task_activation_msg.get())
            task->setActivation(task_activation_msg->data);
    }

    void TaskInterface::task_weight_callback(const TaskWeightMsgPtr msg){
        rt_task_weight_buffer.writeFromNonRT(msg);
    }

    void TaskInterface::task_activation_callback(const TaskActivationMsgPtr msg){
        rt_task_activation_buffer.writeFromNonRT(msg);
    }

    SpatialVelocityTaskInterface::SpatialVelocityTaskInterface(TaskPtr task, 
                                                               RobotModelPtr robot_model, 
                                                               shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, 
                      robot_model, 
                      node)
    {
        reference_subscriber = node->create_subscription<ReferenceMsg>("~/" + task->config.name + "/reference",  SystemDefaultsQoS(), bind(&SpatialVelocityTaskInterface::reference_callback, this, placeholders::_1));
        status_publisher = node->create_publisher<StatusMsg>("~/" + task->config.name + "/status", SystemDefaultsQoS());
        rt_status_publisher = std::make_unique<RTStatusPublisher>(status_publisher);
        reference.twist.setZero();
    }

    void SpatialVelocityTaskInterface::updateReference(){
        reference_msg = *rt_reference_buffer.readFromRT();
        if(reference_msg.get())
            fromROS(*reference_msg, reference);
        dynamic_pointer_cast<SpatialVelocityTask>(task)->setReference(reference.twist);
    }  

    void SpatialVelocityTaskInterface::publishStatus(){
        status.pose = robot_model->pose(dynamic_pointer_cast<SpatialVelocityTask>(task)->tipFrame());
        status.twist = robot_model->twist(dynamic_pointer_cast<SpatialVelocityTask>(task)->tipFrame());
        rt_status_publisher->lock();    
        toROS(status.pose, status.twist, status.acceleration, rt_status_publisher->msg_);
        rt_status_publisher->unlockAndPublish();
    }


    void SpatialVelocityTaskInterface::reference_callback(const ReferenceMsgPtr msg){
        rt_reference_buffer.writeFromNonRT(msg);
    }

    void SpatialVelocityTaskInterface::reset(){
        task->reset();
        reference.twist.setZero();
    }

    /*SpatialAccelerationTaskInterface::SpatialAccelerationTaskInterface(TaskPtr task, 
                                                                       RobotModelPtr robot_model, 
                                                                       shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, 
                      robot_model, 
                      node, 
                      task->config.name + "/reference/" + spatial_acc_names,
                      task->config.name + "/status/" + (pose_names + twist_names)){
    }
    
    void SpatialAccelerationTaskInterface::updateReference(){
        fromRaw(reference_data, reference);
        dynamic_pointer_cast<SpatialAccelerationTask>(task)->setReference(reference);
    }  

    void SpatialAccelerationTaskInterface::updateStatus(){        
        status_pose = robot_model->pose(dynamic_pointer_cast<SpatialAccelerationTask>(task)->tipFrame());
        status_twist = robot_model->twist(dynamic_pointer_cast<SpatialAccelerationTask>(task)->tipFrame());
        toRaw(status_pose, status_twist, status_data);
    }

    CoMVelocityTaskInterface::CoMVelocityTaskInterface(TaskPtr task, 
                                                       RobotModelPtr robot_model, 
                                                       shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, 
                      robot_model, 
                      node, 
                      task->config.name + "/reference/" + linear_vel_names,
                      task->config.name + "/status/" + position_names){
    }
    
    void CoMVelocityTaskInterface::updateReference(){
        dynamic_pointer_cast<CoMVelocityTask>(task)->setReference(reference_data);
    }  

    void CoMVelocityTaskInterface::updateStatus(){        
        status_data = robot_model->centerOfMass().pose.position;
    }    

    CoMAccelerationTaskInterface::CoMAccelerationTaskInterface(TaskPtr task, 
                                                               RobotModelPtr robot_model, 
                                                               shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, 
                      robot_model, 
                      node, 
                      task->config.name + "/reference/" + linear_acc_names,
                      task->config.name + "/status/" + (position_names + linear_vel_names)){
    }
    
    void CoMAccelerationTaskInterface::updateReference(){
        dynamic_pointer_cast<CoMAccelerationTask>(task)->setReference(reference_data);
    }  

    void CoMAccelerationTaskInterface::updateStatus(){        
        status_data.segment(0,3) = robot_model->centerOfMass().pose.position;
        status_data.segment(3,3) = robot_model->centerOfMass().twist.linear;
    }     

    JointVelocityTaskInterface::JointVelocityTaskInterface(TaskPtr task, 
                                                           RobotModelPtr robot_model, 
                                                           shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, 
                      robot_model, 
                      node, 
                      task->config.name + "/reference/" + std::dynamic_pointer_cast<wbc::JointVelocityTask>(task)->jointNames() + "/velocity", 
                      vector<string>()){ 

    }
    
    void JointVelocityTaskInterface::updateReference(){
        dynamic_pointer_cast<JointVelocityTask>(task)->setReference(reference_data);
    }  

    void JointVelocityTaskInterface::updateStatus(){
        // No status need for contact joint tasks (read status directly from hardware interfaces)
    }               

    JointAccelerationTaskInterface::JointAccelerationTaskInterface(TaskPtr task, 
                                                                   RobotModelPtr robot_model, 
                                                                   shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, 
                      robot_model, 
                      node, 
                      task->config.name + "/reference/" + std::dynamic_pointer_cast<wbc::JointVelocityTask>(task)->jointNames() + "/acceleration",
                      vector<string>()){ 
    }
    
    void JointAccelerationTaskInterface::updateReference(){
        dynamic_pointer_cast<JointAccelerationTask>(task)->setReference(reference_data);
    }  

    void JointAccelerationTaskInterface::updateStatus(){
        // No status need for contact joint tasks (read status directly from hardware interfaces)
    }   

    ContactForceTaskInterface::ContactForceTaskInterface(TaskPtr task, 
                                                         RobotModelPtr robot_model, 
                                                         shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, 
                      robot_model, 
                      node, 
                      task->config.name + "/reference/" + force_names,
                      vector<string>()){ 
    }
    
    void ContactForceTaskInterface::updateReference(){
        reference.force = reference_data;
        dynamic_pointer_cast<ContactForceTask>(task)->setReference(reference);
    }  

    void ContactForceTaskInterface::updateStatus(){
        // No status need for contact force tasks (feed forward only)
    }*/                   
}