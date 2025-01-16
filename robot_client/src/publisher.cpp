// 프로그램 실행 후 1번 publish하는 코드

// In terminal: ros2 topic pub -1 /your_pose_topic geometry_msgs/msg/PoseStamped "{header: {stamp: now, frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.4, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
// Panda example: ros2 topic pub /target_position geometry_msgs/msg/PoseStamped "{header: {stamp: now, frame_id: 'base_link'}, pose: {position: {x: 0.3, y: -0.2, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("publisher"); // Create a Node
  
  // Create a publisher for the pose topic
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher = 
      node->create_publisher<geometry_msgs::msg::PoseStamped>("/target_position", 10);

  // Create a PoseStamped message and fill it with data
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = "base_link"; // Replace "base_link" with the appropriate frame ID for your robot
  pose_msg.pose.position.x = 0.1; // Desired position in x-axis
  pose_msg.pose.position.y = 0.1; // Desired position in y-axis
  pose_msg.pose.position.z = 1.0; // Desired position in z-axis
  // 0.2 0.2 0.7~8
  pose_msg.pose.orientation.x = 0.707; // Desired orientation x-component
  pose_msg.pose.orientation.y = 0.707; // Desired orientation y-component
  pose_msg.pose.orientation.z = 0.0; // Desired orientation z-component
  pose_msg.pose.orientation.w = 0.0; // Desired orientation w-component (quaternion)

  // Publish the message
  pose_publisher->publish(pose_msg);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
