#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <robot_control_msgs/msg/joint_command.hpp>

using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node
{
  public:
    JointTrajectoryPublisher() : Node("joint_trajectory_publisher"){
        publisher = this->create_publisher<robot_control_msgs::msg::JointCommand>("setpoint", 10);

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
        auto msg = robot_control_msgs::msg::JointCommand();
        msg.position.resize(joint_names.size());
        msg.velocity.resize(joint_names.size());
        msg.acceleration.resize(joint_names.size());
        for(uint i = 0; i < joint_names.size(); i++){
            msg.position[i] = initial_positions[i] + amplitude*sin(frequency*dt);
            msg.velocity[i] = cos(frequency*dt);
            msg.acceleration[i] = -sin(frequency*dt);
        }
        publisher->publish(msg);
        dt += 0.01;
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<robot_control_msgs::msg::JointCommand>::SharedPtr publisher;
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
