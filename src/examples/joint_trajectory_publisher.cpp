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

        declare_parameter("amplitude", std::vector<double>());
        declare_parameter("frequency", std::vector<double>());
        declare_parameter("joint_names", std::vector<std::string>());

        amplitude       = get_parameter("amplitude").as_double_array();
        frequency       = get_parameter("frequency").as_double_array();
        joint_names     = get_parameter("joint_names").as_string_array();

        assert(amplitude.size() == joint_names.size());
        assert(frequency.size() == joint_names.size());

        initial_positions.resize(joint_names.size());
        for(uint i = 0; i < joint_names.size(); i++){
            declare_parameter("initial_positions." + joint_names[i], 0.0);
            initial_positions[i] = get_parameter("initial_positions." + joint_names[i]).as_double();
        }

        timer = this->create_wall_timer(1ms, std::bind(&JointTrajectoryPublisher::timer_callback, this));
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
            msg.position[i] = initial_positions[i] + amplitude[i]*sin(2*M_PI*frequency[i]*dt);
            msg.velocity[i] = 2*M_PI*frequency[i]*amplitude[i]*cos(2*M_PI*frequency[i]*dt);
            msg.acceleration[i] = -2*M_PI*frequency[i]*2*M_PI*frequency[i]*amplitude[i]*sin(2*M_PI*frequency[i]*dt);
        }
        publisher->publish(msg);
        dt += 0.001;
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<robot_control_msgs::msg::JointCommand>::SharedPtr publisher;
    std::vector<double> amplitude;
    std::vector<double> frequency;
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
