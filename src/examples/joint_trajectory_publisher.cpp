#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node
{
  public:
    JointTrajectoryPublisher() : Node("joint_trajectory_publisher"){
        publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("setpoint", 10);

        declare_parameter("amplitude", 0.1);
        declare_parameter("frequency", 1.0);
        declare_parameter("joint_names", std::vector<std::string>());

        amplitude       = get_parameter("amplitude").as_double();
        frequency       = get_parameter("frequency").as_double();
        joint_names     = get_parameter("joint_names").as_string_array();

        initial_positions.resize(joint_names.size());
        for(uint i = 0; i < joint_names.size(); i++){
            declare_parameter("initial_positions." + joint_names[i], 0.0);
            initial_positions[i] = get_parameter("initial_positions." + joint_names[i]).as_double();
        }

        timer = this->create_wall_timer(10ms, std::bind(&JointTrajectoryPublisher::timer_callback, this));
        dt = 0;
    }

  private:
    void timer_callback()
    {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.points.resize(1);
        msg.joint_names = joint_names;
        msg.points[0].positions.resize(joint_names.size());
        msg.points[0].velocities.resize(joint_names.size());
        msg.points[0].accelerations.resize(joint_names.size());
        for(uint i = 0; i < joint_names.size(); i++){
            msg.points[0].positions[i] = initial_positions[i] + amplitude*sin(frequency*dt);
            msg.points[0].velocities[i] = amplitude*cos(frequency*dt);
            msg.points[0].accelerations[i] = 0;
        }
        publisher->publish(msg);
        dt += 0.01;
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
    double amplitude;
    double frequency;
    std::vector<std::string> joint_names;
    std::vector<double> initial_positions;
    double dt;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
