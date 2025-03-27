#ifndef MOTION_PLANNER__MOTION_PLANNER_HPP_
#define MOTION_PLANNER__MOTION_PLANNER_HPP_

#include "autonomous_motion_planner/planners/a_star.hpp"

#include "grid_map_core/GridMap.hpp"
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

class MotionPlanner : public rclcpp::Node
{
public:
  MotionPlanner();

private:
  // Parameters
  std::string odom_topic_;
  std::string grid_map_topic_;
  std::string planner_type_;

  // Variables
  grid_map::GridMap grid_;

  // Subscriber
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr occ_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &);
  void gridCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr &);
  void timerCallback();

  // Functions
};

#endif // MOTION_PLANNER__MOTION_PLANNER_HPP_
