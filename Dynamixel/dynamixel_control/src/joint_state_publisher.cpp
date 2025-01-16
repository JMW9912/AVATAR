#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>

class JointStatePublisher : public rclcpp::Node {
public:
  JointStatePublisher() : Node("joint_state_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // 타이머를 설정하여 주기적으로 기본값을 퍼블리시
    this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&JointStatePublisher::publishDefaultJointState, this));
      
    RCLCPP_INFO(this->get_logger(), "Timer created to publish joint states every 1 second");
  }

private:
  void publishDefaultJointState() {
    // 기본값으로 joint state 메시지 생성
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->get_clock()->now();
    message.name = {"joint1", "joint2"}; // 관절 이름
    message.position = {50.0, 50.0}; // 기본값

    RCLCPP_INFO(this->get_logger(), "Publishing default joint states:");
    for (size_t i = 0; i < message.position.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  Joint %zu: %f", i + 1, message.position[i]);
    }

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Message published");
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node is starting...");
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node is shutting down...");
  rclcpp::shutdown();
  return 0;
}
