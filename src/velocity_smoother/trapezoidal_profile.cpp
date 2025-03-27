#include "autonomous_motion_planner/velocity_smoother/trapezoidal_profile.hpp"

namespace trapezoidal_profile
{

  double euclideanDistance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
  {
    return (p1 - p2).norm();
  }

  void compute_trapezoidal_profile(motion_planner::msg::Path &path,
                                   const kinematic_limits &params)
  {}

  // Function to compute trapezoidal velocity profile based on distance
  void velocity_assignment(motion_planner::msg::Path &path, double v_init, double v_max,
                           double v_final, double a_max)
  {
    int num_waypoints = waypoints.size();
    std::vector<double> velocity_profile(num_waypoints, 0.0);
    std::vector<double> distance_profile(num_waypoints, 0.0);

    // Compute cumulative distance along the path
    for(int i = 1; i < num_waypoints; ++i)
    {
      distance_profile[i]
        = distance_profile[i - 1] + euclideanDistance(waypoints[i - 1], waypoints[i]);
    }

    double total_distance = distance_profile.back();

    // Find acceleration and deceleration distances
    double accel_dist
      = (v_max * v_max - v_init * v_init) / (2 * a_max); // Using v² = u² + 2as
    double decel_dist = (v_max * v_max - v_final * v_final) / (2 * a_max);
    double cruise_dist = std::max(0.0, total_distance - (accel_dist + decel_dist));

    // Assign velocity based on trapezoidal profile
    for(int i = 0; i < num_waypoints; ++i)
    {
      double d = distance_profile[i];

      if(d <= accel_dist)
      {
        // Acceleration phase
        velocity_profile[i] = std::sqrt(v_init * v_init + 2 * a_max * d);
      }
      else if(d <= accel_dist + cruise_dist)
      {
        // Constant velocity phase
        velocity_profile[i] = v_max;
      }
      else
      {
        // Deceleration phase
        double decel_d = total_distance - d;
        velocity_profile[i]
          = std::sqrt(std::max(0.0, v_final * v_final + 2 * a_max * decel_d));
      }
    }
  }
}
