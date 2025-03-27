#include "autonomous_motion_planner/motion_planner.hpp"

MotionPlanner::MotionPlanner() : Node("motion_planner_node")
{
  // Parameters
  odom_topic_ = declare_parameter<std::string>("odometry_topic", "");
  grid_map_topic_ = declare_parameter<std::string>("occupancy_map_topic", "");
  planner_type_ = declare_parameter<std::string>("planner_type", "");

  if(odom_topic_.empty() || grid_map_topic_.empty())
  {
    RCLCPP_ERROR(get_logger(), "Odometry or Occupany map topic is not assigned");
    return;
  }

  if(planner_type_.empty())
  {
    RCLCPP_ERROR(get_logger(), "Motion planner is not set");
    return;
  }

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 1,
    std::bind(&MotionPlanner::odometryCallback, this, std::placeholders::_1));
  occ_grid_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic_, 1,
    std::bind(&MotionPlanner::gridCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                   std::bind(&MotionPlanner::timerCallback, this));
}

void MotionPlanner::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
  return;
}

void MotionPlanner::gridCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr &msg)
{
  return;
}

void MotionPlanner::timerCallback() { return; }

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlanner>();
  rclcpp::spin(node);
  return 0;
}
