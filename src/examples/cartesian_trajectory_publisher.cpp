#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <wbc_msgs/msg/rigid_body_state.hpp>

using namespace std::chrono_literals;

/**
@brief Publishes a sinudoidal trajectory in Cartesian space. 
*/
class CartesianTrajectoryPublisher : public rclcpp::Node
{
  public:
    CartesianTrajectoryPublisher() : Node("cartesian_trajectory_publisher"){
        publisher_ = this->create_publisher<wbc_msgs::msg::RigidBodyState>("setpoint", 10);

        declare_parameter("amplitude", 0.1);
        declare_parameter("frequency", 1.0);
        declare_parameter("plane", "xy");
        declare_parameter("init_pos_x", 0.0);
        declare_parameter("init_pos_y", 0.0);
        declare_parameter("init_pos_z", 0.0);
        declare_parameter("init_ori_qx", 0.0);
        declare_parameter("init_ori_qy", 0.0);
        declare_parameter("init_ori_qz", 0.0);        
        declare_parameter("init_ori_qw", 1.0);    
         
        amplitude       = get_parameter("amplitude").as_double();
        frequency       = get_parameter("frequency").as_double();
        plane           = get_parameter("plane").as_string();
        init_pos_x      = get_parameter("init_pos_x").as_double();
        init_pos_y      = get_parameter("init_pos_y").as_double();
        init_pos_z      = get_parameter("init_pos_z").as_double();
        init_ori_qx     = get_parameter("init_ori_qx").as_double();
        init_ori_qy     = get_parameter("init_ori_qy").as_double();
        init_ori_qz     = get_parameter("init_ori_qz").as_double();   
        init_ori_qw     = get_parameter("init_ori_qw").as_double();                                                 
        
        timer_ = this->create_wall_timer(10ms, std::bind(&CartesianTrajectoryPublisher::timer_callback, this));
        dt = 0;
    }

  private:
    void timer_callback()
    {
        auto msg = wbc_msgs::msg::RigidBodyState();
        if(plane == "xy"){
	    msg.pose.position.x = init_pos_x + amplitude*sin(2*M_PI*frequency*dt);
            msg.pose.position.y = init_pos_y + amplitude*cos(2*M_PI*frequency*dt);
 	    msg.pose.position.z = init_pos_z;
 	    msg.twist.linear.x = amplitude*cos(2*M_PI*frequency*dt);
     	    msg.twist.linear.y = -amplitude*sin(2*M_PI*frequency*dt);
            msg.twist.linear.z = 0.0;
	}
	else if(plane == "xz"){
	    msg.pose.position.x = init_pos_x + amplitude*sin(2*M_PI*frequency*dt);
            msg.pose.position.y = init_pos_y;
 	    msg.pose.position.z = init_pos_z + amplitude*cos(2*M_PI*frequency*dt);
 	    msg.twist.linear.x = amplitude*cos(2*M_PI*frequency*dt);
     	    msg.twist.linear.y = 0.0;
            msg.twist.linear.z = -amplitude*sin(2*M_PI*frequency*dt);
	}
	else if(plane == "yz"){
	    msg.pose.position.x = init_pos_x;
            msg.pose.position.y = init_pos_y + amplitude*sin(2*M_PI*frequency*dt);
 	    msg.pose.position.z = init_pos_z + amplitude*cos(2*M_PI*frequency*dt);
 	    msg.twist.linear.x = 0.0;
     	    msg.twist.linear.y = amplitude*cos(2*M_PI*frequency*dt);
            msg.twist.linear.z = -amplitude*sin(2*M_PI*frequency*dt);
	}
	else
            throw std::runtime_error("Invalid parameter: plane: " + plane);
            
        msg.pose.orientation.x = init_ori_qx;
        msg.pose.orientation.y = init_ori_qy;
        msg.pose.orientation.z = init_ori_qz;
        msg.pose.orientation.w = init_ori_qw;
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;               
        publisher_->publish(msg);
        dt += 0.01;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<wbc_msgs::msg::RigidBodyState>::SharedPtr publisher_;
    double dt;
    double amplitude;
    double frequency;
    std::string plane;
    double init_pos_x;
    double init_pos_y;
    double init_pos_z;        
    double init_ori_qx;
    double init_ori_qy;
    double init_ori_qz;
    double init_ori_qw;            
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
