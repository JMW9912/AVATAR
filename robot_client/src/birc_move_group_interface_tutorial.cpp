// 동일하거나 일정 범위 이하로 움직이면 스킵하는 코드
//     void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//     target_pose = msg->pose;
//     double position_error = std::abs(target_pose.position.x - temp.position.x) +
//                             std::abs(target_pose.position.y - temp.position.y) +
//                             std::abs(target_pose.position.z - temp.position.z);

//     // double orientation_error = std::abs(target_pose.orientation.x - temp.orientation.x) +
//     //                            std::abs(target_pose.orientation.y - temp.orientation.y) +
//     //                            std::abs(target_pose.orientation.z - temp.orientation.z) +
//     //                            std::abs(target_pose.orientation.w - temp.orientation.w);

//     if (position_error < 0.01) {
//         RCLCPP_INFO(LOGGER, "Skip");
//         return;
//     }

//     move_group.setPoseTarget(target_pose);
//     temp = target_pose;
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//         RCLCPP_INFO(LOGGER, "Planning succeeded");
//         move_group.execute(my_plan);
//         move_group.move();
//     } else {
//         RCLCPP_ERROR(LOGGER, "Planning failed!");
//     }
// } 



#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

static const std::string PLANNING_GROUP = "arm";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");


class MoveGroupDemo : public rclcpp::Node
{
  public:
    MoveGroupDemo();
    moveit::planning_interface::MoveGroupInterface move_group;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::Pose temp;

  private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      target_pose = msg->pose;
      // if(target_pose == temp){
      //   return;
      // }

      move_group.setPoseTarget(target_pose);
      temp = target_pose;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success){
      RCLCPP_INFO(LOGGER, "Planning succeeded");
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