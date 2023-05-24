#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

const std::string joint_names[7] = {
  "joint_0",
  "joint_1",
  "joint_2",
  "joint_3",
  "joint_4",
  "joint_5",
  "joint_6",
};

class JointTrajectoryPublisher : public rclcpp::Node
{
  public:
    JointTrajectoryPublisher() : Node("joint_trajectory_publisher"){
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("setpoint", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&JointTrajectoryPublisher::timer_callback, this));
        dt = 0;
    }

  private:
    void timer_callback()
    {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.points.resize(1);
        for(uint i = 0; i < 7; i++){
            msg.joint_names.push_back(joint_names[i]);
            msg.points[0].positions.push_back(0);
            msg.points[0].velocities.push_back(0);
            msg.points[0].accelerations.push_back(0);
        }
        msg.points[0].positions[3] = sin(dt);
        msg.points[0].velocities[3] = cos(dt);
        publisher_->publish(msg);
        dt += 0.01;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    double dt;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
