#ifndef TRAPEZOIDAL_PROFILE__TRAPEZOIDAL_PROFILE_HPP_
#define TRAPEZOIDAL_PROFILE__TRAPEZOIDAL_PROFILE_HPP_

#include <motion_planner/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <vector>

namespace trapezoidal_profile
{

  struct kinematic_limits
  {
    double vel_limit;
    double acc_limit, lat_acc_limit;
  };

  motion_planner::msg::Path compute_trapezoidal_profile(const kinematic_limits &);
}

#endif // TRAPEZOIDAL_PROFILE__TRAPEZOIDAL_PROFILE_HPP_
