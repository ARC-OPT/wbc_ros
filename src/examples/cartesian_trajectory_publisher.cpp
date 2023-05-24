#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <wbc_msgs/msg/rigid_body_state.hpp>

using namespace std::chrono_literals;


class CartesianTrajectoryPublisher : public rclcpp::Node
{
  public:
    CartesianTrajectoryPublisher() : Node("cartesian_trajectory_publisher"){
        publisher_ = this->create_publisher<wbc_msgs::msg::RigidBodyState>("setpoint", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&CartesianTrajectoryPublisher::timer_callback, this));
        dt = 0;
    }

  private:
    void timer_callback()
    {
        auto msg = wbc_msgs::msg::RigidBodyState();
        msg.pose.position.x = 0.1*sin(dt);
        msg.pose.position.y = 0.1*cos(dt);
        msg.pose.position.z = 1.1;
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = -0.7071707508454255;
        msg.pose.orientation.z = 0;
        msg.pose.orientation.w = 0.7070427866905058;
        publisher_->publish(msg);
        dt += 0.01;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<wbc_msgs::msg::RigidBodyState>::SharedPtr publisher_;
    double dt;
    double period;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
