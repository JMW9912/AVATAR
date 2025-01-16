// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// // Callback function for the subscriber
// void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//   rclcpp::Node::SharedPtr node = rclcpp::Node::GetCurrentNode();

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "arm");

//   // Set a target Pose
//   geometry_msgs::msg::Pose target_pose_arm;
//   target_pose_arm.orientation = msg->pose.orientation;
//   target_pose_arm.position = msg->pose.position;
//   move_group_interface.setPoseTarget(target_pose_arm);

//   // Create a plan to that target pose
//   auto [success, plan] = [&move_group_interface]() {
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     auto ok = static_cast<bool>(move_group_interface.plan(plan));
//     return std::make_pair(ok, plan);
//   }();

//   // Execute the plan
//   if (success) {
//     move_group_interface.execute(plan);
//   } else {
//     RCLCPP_ERROR(rclcpp::get_logger("move_ee"), "Planning failed!");
//   }
// }

// int main(int argc, char *argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>(
//       "move_ee",
//       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   // Create a ROS logger
//   auto logger = rclcpp::get_logger("move_ee");

//   // Create the subscriber to receive target Pose
//   auto subscriber = node->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/your_pose_topic", // Replace "your_pose_topic" with the actual topic name
//       10,
//       poseCallback);

//   // Spin the node and start processing callbacks
//   rclcpp::spin(node);

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }


// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// class MoveEE : public rclcpp::Node
// {
// public:
//   MoveEE() : Node("move_ee")
//   {
//     // Create a subscriber to receive target Pose
//     subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//         "/your_pose_topic", // Replace "your_pose_topic" with the actual topic name
//         10,
//         std::bind(&MoveEE::poseCallback, this, std::placeholders::_1));
//   }

// private:
//   void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     // Create the MoveIt MoveGroup Interface
//     using moveit::planning_interface::MoveGroupInterface;
//     // auto move_group_interface = MoveGroupInterface("arm");
//     MoveGroupInterface move_group_interface("arm");

//     // Set a target Pose
//     geometry_msgs::msg::Pose target_pose_arm;
//     target_pose_arm.orientation = msg->pose.orientation;
//     target_pose_arm.position = msg->pose.position;
//     move_group_interface.setPoseTarget(target_pose_arm);

//     // Create a plan to that target pose
//     auto [success, plan] = [&move_group_interface]() {
//       moveit::planning_interface::MoveGroupInterface::Plan plan;
//       auto ok = static_cast<bool>(move_group_interface.plan(plan));
//       return std::make_pair(ok, plan);
//     }();

//     // Execute the plan
//     if (success) {
//       move_group_interface.execute(plan);
//     } else {
//       RCLCPP_ERROR(get_logger(), "Planning failed!");
//     }
//   }

//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MoveEE>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MoveEE : public rclcpp::Node
{
public:
  MoveEE() : Node("move_ee")
  {


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // Create a subscriber to receive target Pose
    subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/your_pose_topic", // Replace "your_pose_topic" with the actual topic name
        10,
        std::bind(&MoveEE::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    // moveit::planning_interface::MoveGroupInterface::Options options("arm"); // Use the correct constructor

    // MoveGroupInterface move_group_interface(options);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("birc_move_group_interface_tutorial", node_options);
    auto move_group_interface = MoveGroupInterface(move_group_node, "arm");

    // Set a target Pose
    geometry_msgs::msg::Pose target_pose_arm;
    target_pose_arm.orientation = msg->pose.orientation;
    target_pose_arm.position = msg->pose.position;
    move_group_interface.setPoseTarget(target_pose_arm);

    // Create a plan to that target pose
    auto [success, plan] = [&move_group_interface]() {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto ok = static_cast<bool>(move_group_interface.plan(plan));
      return std::make_pair(ok, plan);
    }();

    // Execute the plan
    if (success) {
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(get_logger(), "Planning failed!");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveEE>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
