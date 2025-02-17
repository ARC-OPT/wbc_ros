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
                                 shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
            task(task), 
            node(node){  
        task_weight_subscriber = node->create_subscription<TaskWeightMsg>("~/" + task->config.name + "/task_weights", SystemDefaultsQoS(), bind(&TaskInterface::task_weight_callback, this, placeholders::_1));
        task_activation_subscriber = node->create_subscription<TaskActivationMsg>("~/" + task->config.name + "/activation", SystemDefaultsQoS(), bind(&TaskInterface::task_activation_callback, this, placeholders::_1));
    }
    void TaskInterface::updateTaskWeights(){
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
                                                               wbc::CartesianPosPDControllerPtr controller,
                                                               RobotModelPtr robot_model, 
                                                               shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, node),
        controller(controller),
        robot_model(robot_model){
        setpoint_subscriber = node->create_subscription<SetpointMsg>("~/" + task->config.name + "/setpoint",  SystemDefaultsQoS(), bind(&SpatialVelocityTaskInterface::setpoint_callback, this, placeholders::_1));
        control_output.setZero();
    }
    void SpatialVelocityTaskInterface::updateTask(){
        updateTaskWeights();
        setpoint_msg = *rt_setpoint_buffer.readFromRT();
        if(setpoint_msg.get()){
            fromROS(*setpoint_msg, setpoint);
            pose = robot_model->pose(dynamic_pointer_cast<SpatialVelocityTask>(task)->tipFrame());            
            control_output = controller->update(setpoint.pose, setpoint.twist, pose);
            dynamic_pointer_cast<SpatialVelocityTask>(task)->setReference(control_output);
        }
    }  
    void SpatialVelocityTaskInterface::setpoint_callback(const SetpointMsgPtr msg){
        rt_setpoint_buffer.writeFromNonRT(msg);
    }

    SpatialAccelerationTaskInterface::SpatialAccelerationTaskInterface(TaskPtr task, 
                                                                       wbc::CartesianPosPDControllerPtr controller,                                                                       
                                                                       RobotModelPtr robot_model, 
                                                                       shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task,  node),
        controller(controller),
        robot_model(robot_model){
        setpoint_subscriber = node->create_subscription<SetpointMsg>("~/" + task->config.name + "/setpoint",  SystemDefaultsQoS(), bind(&SpatialAccelerationTaskInterface::setpoint_callback, this, placeholders::_1));
        control_output.setZero();
    }

    void SpatialAccelerationTaskInterface::updateTask(){
        setpoint_msg = *rt_setpoint_buffer.readFromRT();
        if(setpoint_msg.get()){
            fromROS(*setpoint_msg, setpoint);
            pose = robot_model->pose(dynamic_pointer_cast<SpatialAccelerationTask>(task)->tipFrame());
            twist = robot_model->twist(dynamic_pointer_cast<SpatialAccelerationTask>(task)->tipFrame());
            control_output = controller->update(setpoint.pose, setpoint.twist, setpoint.acceleration, pose, twist);   
            dynamic_pointer_cast<SpatialAccelerationTask>(task)->setReference(control_output);     
        }
    }  

    void SpatialAccelerationTaskInterface::setpoint_callback(const SetpointMsgPtr msg){
        rt_setpoint_buffer.writeFromNonRT(msg);
    }

    /*CoMVelocityTaskInterface::CoMVelocityTaskInterface(wbc::TaskPtr task, 
                                                       wbc::ControllerPtr controller,
                                                       wbc::RobotModelPtr robot_model, 
                                                       std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, 
                      controller,
                      robot_model, 
                      node){
        setpoint_subscriber = node->create_subscription<SetpointMsg>("~/" + task->config.name + "/setpoint",  SystemDefaultsQoS(), bind(&CoMVelocityTaskInterface::setpoint_callback, this, placeholders::_1));
        control_output.setZero();                        
    }
    
    void CoMVelocityTaskInterface::updateTask(){
        setpoint_msg = *rt_setpoint_buffer.readFromRT();
        if(setpoint_msg.get()){
            fromROS(*setpoint_msg, setpoint);
            pose = robot_model->pose(dynamic_pointer_cast<SpatialVelocityTask>(task)->tipFrame());
            control_output = dynamic_pointer_cast<CartesianPosPDController>(controller)->update(setpoint.pose, 
                                                                                                setpoint.twist, 
                                                                                                pose);
        }
        dynamic_pointer_cast<CoMVelocityTask>(task)->setReference(control_output.linear);
    }  

    void CoMVelocityTaskInterface::setpoint_callback(const SetpointMsgPtr msg){
        rt_setpoint_buffer.writeFromNonRT(msg);
    }

    void CoMVelocityTaskInterface::reset(){
        task->reset();
        control_output.setZero();
    }*/

    /*CoMAccelerationTaskInterface::CoMAccelerationTaskInterface(TaskPtr task, 
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
    }*/     

    JointVelocityTaskInterface::JointVelocityTaskInterface(wbc::TaskPtr task, 
                                                           wbc::JointPosPDControllerPtr controller,
                                                           wbc::RobotModelPtr robot_model, 
                                                           std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, node),
        controller(controller),
        robot_model(robot_model){ 
        setpoint_subscriber = node->create_subscription<SetpointMsg>("~/" + task->config.name + "/setpoint",  SystemDefaultsQoS(), bind(&JointVelocityTaskInterface::setpoint_callback, this, placeholders::_1));
        control_output.setZero();
    }
    
    void JointVelocityTaskInterface::updateTask(){
        setpoint_msg = *rt_setpoint_buffer.readFromRT();
        if(setpoint_msg.get()){
            fromROS(*setpoint_msg, setpoint);
            joint_state.position = robot_model->jointState().position;
            control_output = controller->update(setpoint.position, setpoint.velocity, joint_state.position);   
            dynamic_pointer_cast<JointVelocityTask>(task)->setReference(control_output);     
        }
    }  

    void JointVelocityTaskInterface::setpoint_callback(const SetpointMsgPtr msg){
        rt_setpoint_buffer.writeFromNonRT(msg);
    }

    JointAccelerationTaskInterface::JointAccelerationTaskInterface(wbc::TaskPtr task, 
                                                                   wbc::JointPosPDControllerPtr controller,
                                                                   wbc::RobotModelPtr robot_model, 
                                                                   std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) : 
        TaskInterface(task, node),
        controller(controller),
        robot_model(robot_model){ 
        setpoint_subscriber = node->create_subscription<SetpointMsg>("~/" + task->config.name + "/setpoint",  SystemDefaultsQoS(), bind(&JointAccelerationTaskInterface::setpoint_callback, this, placeholders::_1));
        control_output.setZero();
    }
    
    void JointAccelerationTaskInterface::updateTask(){
        setpoint_msg = *rt_setpoint_buffer.readFromRT();
        if(setpoint_msg.get()){
            fromROS(*setpoint_msg, setpoint);
            joint_state.position = robot_model->jointState().position;
            joint_state.velocity = robot_model->jointState().velocity;
            control_output = controller->update(setpoint.position, setpoint.velocity, setpoint.acceleration, joint_state.position, joint_state.velocity);   
            dynamic_pointer_cast<JointAccelerationTask>(task)->setReference(control_output);   
        }
    }  

    void JointAccelerationTaskInterface::setpoint_callback(const SetpointMsgPtr msg){
        rt_setpoint_buffer.writeFromNonRT(msg);
    } 

    /*ContactForceTaskInterface::ContactForceTaskInterface(TaskPtr task, 
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