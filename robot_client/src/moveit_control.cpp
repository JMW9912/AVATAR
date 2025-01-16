// 참고: https://github.com/GhoshRitika/CopyCat/blob/504d528d5548d2f404059cecf6a59de7bdb8c696/allegro_driver/src/plan_hand.cpp
// 참고 후보: https://github.com/WilliamLinxw/ROS2/blob/efc800e861b6761671a223e947c45d902d1d8aca/ws_ros2_moveit2/src/moveit2/moveit_demo_nodes/run_move_group/src/calligraphy_sub.cpp#L20
// 그냥 오리지널
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const std::string PLANNING_GROUP = "arm";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");


class MoveGroupDemo : public rclcpp::Node
{
  public:
    MoveGroupDemo();
    moveit::planning_interface::MoveGroupInterface move_group;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    geometry_msgs::msg::Pose target_pose;
    // rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_subscriber_;

  private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      target_pose.orientation = msg->pose.orientation;
      target_pose.position = msg->pose.position;
      // target_pose.position = msg->pose.position;
      // target_pose.position.x = msg->pose.position.x + 0.3;
      // target_pose.position.y = msg->pose.position.y + 0.4;
      // target_pose.position.z = msg->pose.position.z + 1.0;
      move_group.setPoseTarget(target_pose);
      // move_group.setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success){
      RCLCPP_INFO(LOGGER, "Planning succeeded!");
      move_group.execute(my_plan);
      move_group.move();
      }
      else{
      RCLCPP_ERROR(LOGGER, "Planning failed!");
      }
    }
};

MoveGroupDemo::MoveGroupDemo(): Node("move_group"), move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), PLANNING_GROUP)
{
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_position",
    10,
    std::bind(&MoveGroupDemo::poseCallback, this, std::placeholders::_1));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto move_group_demo_node = std::make_shared<MoveGroupDemo>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_demo_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}