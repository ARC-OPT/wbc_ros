#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <robot_control_msgs/msg/rigid_body_state.hpp>

using namespace std::chrono_literals;

/**
@brief Publishes a pose in Cartesian space.
*/
class CartesianPosePublisher : public rclcpp::Node
{
  public:
    CartesianPosePublisher() : Node("cartesian_pose_publisher"){
        publisher_ = this->create_publisher<robot_control_msgs::msg::RigidBodyState>("setpoint", 10);

        declare_parameter("pos_x", 0.0);
        declare_parameter("pos_y", 0.0);
        declare_parameter("pos_z", 1.0);
        declare_parameter("ori_qx", 0.0);
        declare_parameter("ori_qy", 0.0);
        declare_parameter("ori_qz", 0.0);
        declare_parameter("ori_qw", 1.0);

        pos_x      = get_parameter("pos_x").as_double();
        pos_y      = get_parameter("pos_y").as_double();
        pos_z      = get_parameter("pos_z").as_double();
        ori_qx     = get_parameter("ori_qx").as_double();
        ori_qy     = get_parameter("ori_qy").as_double();
        ori_qz     = get_parameter("ori_qz").as_double();
        ori_qw     = get_parameter("ori_qw").as_double();

        timer_ = this->create_wall_timer(10ms, std::bind(&CartesianPosePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto msg = robot_control_msgs::msg::RigidBodyState();
        msg.pose.position.x = pos_x;
        msg.pose.position.y = pos_y;
        msg.pose.position.z = pos_z;
        msg.pose.orientation.x = ori_qx;
        msg.pose.orientation.y = ori_qy;
        msg.pose.orientation.z = ori_qz;
        msg.pose.orientation.w = ori_qw;
        publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<robot_control_msgs::msg::RigidBodyState>::SharedPtr publisher_;
    double pos_x;
    double pos_y;
    double pos_z;
    double ori_qx;
    double ori_qy;
    double ori_qz;
    double ori_qw;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
