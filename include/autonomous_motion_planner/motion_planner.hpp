#ifndef MOTION_PLANNER__MOTION_PLANNER_HPP_
#define MOTION_PLANNER__MOTION_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
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

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &);
  void gridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &);
  void timerCallback();

  // Functions
};

#endif // MOTION_PLANNER__MOTION_PLANNER_HPP_
