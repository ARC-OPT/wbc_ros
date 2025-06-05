#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robot_control_msgs/msg/robot_state.hpp>

using namespace rclcpp;
using namespace std;

class JointStateConverter : public Node{
       
    using RobotStateMsg = robot_control_msgs::msg::JointState;
    using RobotStateMsgPtr = std::shared_ptr<RobotStateMsg>;
    using RobotStateSubscription = rclcpp::Subscription<RobotStateMsg>::SharedPtr;

    using JointStateMsg = sensor_msgs::msg::JointState;
    using JointStatePublisher = rclcpp::Publisher<JointStateMsg>::SharedPtr;

    public:
        JointStateConverter() : Node("joint_state_converter"){
            joint_state_publisher = this->create_publisher<JointStateMsg>("/joint_states", 10);
            robot_state_subscriber = this->create_subscription<RobotStateMsg>("~/joint_state", rclcpp::SystemDefaultsQoS(), 
                bind(&JointStateConverter::robot_state_callback, this, placeholders::_1));

            this->declare_parameter("joint_names", vector<string>());
            joint_state.name = this->get_parameter("joint_names").as_string_array(); 


            RCLCPP_INFO(get_logger(), "Joint State Converter is running");
        }

        void robot_state_callback(const RobotStateMsgPtr msg){ 
            joint_state.header.stamp = this->get_clock()->now();
            joint_state.position = msg->position;
            joint_state.velocity = msg->velocity;
            joint_state.effort = msg->effort;

            joint_state_publisher->publish(joint_state);
        }

    protected:
        RobotStateSubscription robot_state_subscriber;
        JointStatePublisher joint_state_publisher;
        JointStateMsg joint_state;
};

int main(int argc, char * argv[])
{
  init(argc, argv);
  spin(std::make_shared<JointStateConverter>());
  shutdown();
  return 0;
}
