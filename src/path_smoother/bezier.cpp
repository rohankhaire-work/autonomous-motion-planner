#include "autonomous_motion_planner/path_smoother/bezier.hpp"

namespace bezier_smoothing
{
  // Cubic Bézier interpolation function
  Eigen::Vector2d
  bezierInterpolate(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1,
                    const Eigen::Vector2d &p2, const Eigen::Vector2d &p3, double t)
  {
    double u = 1.0 - t;
    double tt = t * t;
    double uu = u * u;
    double uuu = uu * u;
    double ttt = tt * t;

    return (uuu * p0) + (3 * uu * t * p1) + (3 * u * tt * p2) + (ttt * p3);
  }

  // Bézier smoothing function
  std::vector<geometry_msgs::msg::Point>
  bezierSmoothing(const std::vector<geometry_msgs::msg::Point> &waypoints,
                  double resolution, int smooth_length)
  {
    std::vector<geometry_msgs::msg::Point> smoothedPath;

    if(waypoints.size() < 4)
      return waypoints;

    // smooth first 20
    int trim_size = std::min(waypoints.size(), smooth_length);
    for(size_t i = 0; i < trim_size - 3; i += 3)
    {
      Eigen::Vector2d p0(waypoints[i].x, waypoints[i].y);
      Eigen::Vector2d p1(waypoints[i + 1].x, waypoints[i + 1].y);
      Eigen::Vector2d p2(waypoints[i + 2].x, waypoints[i + 2].y);
      Eigen::Vector2d p3(waypoints[i + 3].x, waypoints[i + 3].y);

      // Compute the total segment length
      double segment_length = euclideanDistance(p0, p1) + euclideanDistance(p1, p2)
                              + euclideanDistance(p2, p3);

      // Determine number of samples dynamically based on segment length
      int num_samples = std::max(1, static_cast<int>(round(segment_length / resolution)));

      for(int j = 0; j <= num_samples; ++j)
      {
        double t = static_cast<double>(j) / num_samples;
        Eigen::Vector2d interpolated = bezierInterpolate(p0, p1, p2, p3, t);

        geometry_msgs::msg::Point smooth_point;
        smooth_point.x = interpolated.x();
        smooth_point.y = interpolated.y();
        smoothedPath.push_back(smooth_point);
      }
    }

    return smoothedPath;
  }

  double euclideanDistance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
  {
    return (p1 - p2).norm();
  }
}
