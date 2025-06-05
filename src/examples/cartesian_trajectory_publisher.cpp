#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <robot_control_msgs/msg/rigid_body_state.hpp>

using namespace std::chrono_literals;

/**
@brief Publishes a sinudoidal trajectory in Cartesian space.
*/
class CartesianTrajectoryPublisher : public rclcpp::Node
{
  public:
    CartesianTrajectoryPublisher() : Node("cartesian_trajectory_publisher"){
        publisher_ = this->create_publisher<robot_control_msgs::msg::RigidBodyState>("setpoint", 10);

        declare_parameter("amplitude_x", 0.1);
        declare_parameter("amplitude_y", 0.1);
        declare_parameter("amplitude_z", 0.1);
        declare_parameter("phase_shift_x", 0.0);
        declare_parameter("phase_shift_y", 0.0);
        declare_parameter("phase_shift_z", 0.0);
        declare_parameter("frequency", 1.0);
        declare_parameter("plane", "xy");
        declare_parameter("init_pos_x", 0.0);
        declare_parameter("init_pos_y", 0.0);
        declare_parameter("init_pos_z", 0.0);
        declare_parameter("init_ori_qx", 0.0);
        declare_parameter("init_ori_qy", 0.0);
        declare_parameter("init_ori_qz", 0.0);
        declare_parameter("init_ori_qw", 1.0);

        amplitude_x     = get_parameter("amplitude_x").as_double();
        amplitude_y     = get_parameter("amplitude_y").as_double();
        amplitude_z     = get_parameter("amplitude_z").as_double();
        phase_shift_x   = get_parameter("phase_shift_x").as_double();
        phase_shift_y   = get_parameter("phase_shift_y").as_double();
        phase_shift_z   = get_parameter("phase_shift_z").as_double();
        frequency       = get_parameter("frequency").as_double();
        plane           = get_parameter("plane").as_string();
        init_pos_x      = get_parameter("init_pos_x").as_double();
        init_pos_y      = get_parameter("init_pos_y").as_double();
        init_pos_z      = get_parameter("init_pos_z").as_double();
        init_ori_qx     = get_parameter("init_ori_qx").as_double();
        init_ori_qy     = get_parameter("init_ori_qy").as_double();
        init_ori_qz     = get_parameter("init_ori_qz").as_double();
        init_ori_qw     = get_parameter("init_ori_qw").as_double();

        timer_ = this->create_wall_timer(1ms, std::bind(&CartesianTrajectoryPublisher::timer_callback, this));
        dt = 0;
    }

  private:
    void timer_callback()
    {
        auto msg = robot_control_msgs::msg::RigidBodyState();
        if(plane == "xy"){
            msg.pose.position.x = init_pos_x + amplitude_x*sin(2*M_PI*frequency*dt + phase_shift_x);
            msg.pose.position.y = init_pos_y + amplitude_y*cos(2*M_PI*frequency*dt + phase_shift_y);
            msg.pose.position.z = init_pos_z;
            msg.twist.linear.x = amplitude_x*cos(2*M_PI*frequency*dt + phase_shift_x);
            msg.twist.linear.y = -amplitude_y*sin(2*M_PI*frequency*dt + phase_shift_y);
            msg.twist.linear.z = 0.0;
            msg.acceleration.linear.x = -amplitude_x*sin(2*M_PI*frequency*dt + phase_shift_x);
            msg.acceleration.linear.y = -amplitude_y*cos(2*M_PI*frequency*dt + phase_shift_y);
            msg.acceleration.linear.z = 0.0;
        }
        else if(plane == "xz"){
            msg.pose.position.x = init_pos_x + amplitude_x*sin(2*M_PI*frequency*dt + phase_shift_x);
            msg.pose.position.y = init_pos_y;
            msg.pose.position.z = init_pos_z + amplitude_z*cos(2*M_PI*frequency*dt + phase_shift_z);
            msg.twist.linear.x = amplitude_x*2*M_PI*frequency*cos(2*M_PI*frequency*dt + phase_shift_x);
            msg.twist.linear.y = 0.0;
            msg.twist.linear.z = -amplitude_z*2*M_PI*frequency*sin(2*M_PI*frequency*dt + phase_shift_z);
            msg.acceleration.linear.x = -amplitude_z*2*M_PI*frequency*2*M_PI*frequency*sin(2*M_PI*frequency*dt + phase_shift_x);
            msg.acceleration.linear.y = 0.0;
            msg.acceleration.linear.z = -amplitude_z*2*M_PI*frequency*2*M_PI*frequency*cos(2*M_PI*frequency*dt + phase_shift_z);
        }
        else if(plane == "yz"){
            msg.pose.position.x = init_pos_x;
            msg.pose.position.y = init_pos_y + amplitude_y*sin(2*M_PI*frequency*dt + phase_shift_y);
            msg.pose.position.z = init_pos_z + amplitude_z*cos(2*M_PI*frequency*dt + phase_shift_z);
            msg.twist.linear.x = 0.0;
            msg.twist.linear.y = cos(2*M_PI*frequency*dt + phase_shift_y);
            msg.twist.linear.z = -sin(2*M_PI*frequency*dt + phase_shift_z);
            msg.acceleration.linear.x = 0.0;
            msg.acceleration.linear.y = -sin(2*M_PI*frequency*dt + phase_shift_y);
            msg.acceleration.linear.z = -cos(2*M_PI*frequency*dt + phase_shift_z);
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
        msg.acceleration.angular.x = 0.0;
        msg.acceleration.angular.y = 0.0;
        msg.acceleration.angular.z = 0.0;
        publisher_->publish(msg);
        dt += 0.001;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<robot_control_msgs::msg::RigidBodyState>::SharedPtr publisher_;
    double dt;
    double amplitude_x;
    double amplitude_y;
    double amplitude_z;
    double phase_shift_x;
    double phase_shift_y;
    double phase_shift_z;
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
