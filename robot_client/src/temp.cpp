#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class MoveGroupDemo : public rclcpp::Node
{
public:
  MoveGroupDemo() : Node("move_group_demo")
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);


    auto move_group_node = rclcpp::Node::make_shared("birc_move_group_interface_tutorial", node_options);

    // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");
    // planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;




    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/your_pose_topic", // Replace with your topic name
        10,
        std::bind(&MoveGroupDemo::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation = msg->pose.orientation;
    target_pose.position = msg->pose.position;
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  
    if (success){
      RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) succeeded");
      move_group_->execute(my_plan);
    }else{
    RCLCPP_ERROR(LOGGER, "Planning failed!");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto move_group_demo_node = std::make_shared<MoveGroupDemo>();
  rclcpp::spin(move_group_demo_node);
  rclcpp::shutdown();
  return 0;
}




#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class MoveGroupDemo : public rclcpp::Node
{
public:
  MoveGroupDemo() : Node("move_group_demo")
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/your_pose_topic", // Replace with your topic name
        10,
        std::bind(&MoveGroupDemo::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation = msg->pose.orientation;
    target_pose.position = msg->pose.position;
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
    if (success)
    {
      RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) succeeded");
      move_group_->execute(my_plan);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Planning failed!");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto move_group_demo_node = std::make_shared<MoveGroupDemo>();
  rclcpp::spin(move_group_demo_node);
  rclcpp::shutdown();
  return 0;
}


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("birc_move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 1.0;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move(); 

  rclcpp::shutdown();
  return 0;
}




#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("birc_move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub = move_group_node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10,
      [&move_group](const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) {
        move_group.setPoseTarget(pose_msg->pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        move_group.move();
      });



  rclcpp::shutdown();
  return 0;
}




// allegro
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>


#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
static const std::string PLANNING_GROUP = "allegro_hand";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

class PlanHand : public rclcpp::Node
{
  public:
    PlanHand();
    moveit::planning_interface::MoveGroupInterface move_group;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_sub_;
    std::vector<double> joint_group_positions;
  

  private:
    void target_pose_callback(const std_msgs::msg::Float64MultiArray &msg);

};

PlanHand::PlanHand(): Node("plan_hand"), move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), PLANNING_GROUP)
{
  RCLCPP_INFO(LOGGER, "INITIALIZING!");
  target_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("Joint_angles", 10, std::bind(&PlanHand::target_pose_callback, this, std::placeholders::_1));
  rclcpp::sleep_for(5s);
  joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.5235, 0.3665, 0.8901, 0.4886, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  this->move_group.setJointValueTarget(joint_group_positions);
  this->move_group.move();
  rclcpp::sleep_for(5s);
}

void PlanHand::target_pose_callback(const std_msgs::msg::Float64MultiArray &msg){
  joint_group_positions = msg.data;
  RCLCPP_INFO(LOGGER, "Received Joint angles");
  bool within_bounds = this->move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }
  move_group.setMaxVelocityScalingFactor(0.85);
  move_group.setMaxAccelerationScalingFactor(0.85);
  this->move_group.move();
}

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto move_group_node = std::make_shared<PlanHand>();
  RCLCPP_INFO(LOGGER, "In main");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}


//caligraphy
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>

#include<vector>
#include<string.h>
#include<algorithm>

#include <fstream>

#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
moveit::planning_interface::MoveGroupInterface* move_group;
std::vector<geometry_msgs::msg::Pose> waypoints;

class MinimalSubscriber : public rclcpp::Node{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "arm_strokes", 1000, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
    {
      
      // Print out the message
      for (uint i = 0; i < msg->data.size(); i++){
        cout << msg->data[i] << ", ";
      }
      cout << endl;

      // Create data structure of Pose and an empty vector for storing the waypoints for Cartesian Planning
      geometry_msgs::msg::Pose target_pose;

      // Now, we call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking move_group
      // to actually move the robot.

   
      if((msg->data[0]==0) && (msg->data[1]==0) && (msg->data[2]==0))
      {
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        cout << "size of waypoints:" << waypoints.size() << endl;
        double fraction = (*move_group).computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian pamove_groupth) (%.2f%% acheived)", fraction * 100.0);
        // You can execute a trajectory like this.
        bool execute = true;
        if(execute == true)
        {
          (*move_group).execute(trajectory);
          waypoints.clear();
        }
      }
      else
      {
        target_pose.position.x = msg->data[0];
        target_pose.position.y = msg->data[1];
        target_pose.position.z = msg->data[2];
        waypoints.push_back(target_pose);
      }
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  static const std::string PLANNING_GROUP = "panda_arm";

  move_group = new moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  // const moveit::core::JointModelGroup* joint_model_group =
  //     (*move_group).getRobotModel()->getJointModelGroup(PLANNING_GROUP);
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", (*move_group).getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", (*move_group).getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy((*move_group).getJointModelGroupNames().begin(), (*move_group).getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}