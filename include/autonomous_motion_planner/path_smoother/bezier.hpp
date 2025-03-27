#ifndef BEZIER__BEZIER_HPP_
#define BEZIER__BEZIER_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace bezier_smoothing
{
  Eigen::Vector2d
  bezier_interpolation(const Eigen::Vector2d &, const Eigen::Vector2d &,
                       const Eigen::Vector2d &, const Eigen::Vector2d &, double);

  std::vector<geometry_msgs::msg::Point>
  bezier_smoothing(const std::vector<geometry_msgs::msg::Point> &, double, int);

  double euclideanDistance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
}

#endif // BEZIER__BEZIER_HPP_
