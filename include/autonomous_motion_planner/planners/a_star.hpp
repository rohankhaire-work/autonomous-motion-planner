#ifndef A_STAR__A_STAR_HPP_
#define A_STAR__A_STAR_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <array>
#include <vector>
#include <queue>
#include <memory>

struct Node
{
  int x, y;
  bool visited;
  std::shared_ptr<Node> parent;
  double hc;
  double gc;

  double cost() const { return gc + hc; }

  Node() : x(0), y(0), visited(false), parent(nullptr), hc(0.0), gc(0.0) {}
  Node(int x_, int y_) : x(x_), y(y_), parent(nullptr), hc(0.0), gc(0.0) {}
};

struct compareOP
{
  bool operator()(const std::shared_ptr<Node> n1, const std::shared_ptr<Node> n2)
  {
    return n1->cost() > n2->cost();
  }
};

class AStarSearch
{
public:
  AStarSearch();
  std::vector<geometry_msgs::msg::Point>
  plan(const geometry_msgs::msg::Point &, const grid_map::GridMap &);
  void setMapParams(int, int, double, const std::string &);

private:
  int map_size_x_, map_size_y_;
  double grid_resolution_;
  std::string layer_name_;
  std::vector<std::vector<int>> neighbors
    = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  double heuristicCost(const std::shared_ptr<Node> &, const std::shared_ptr<Node> &,
                       const grid_map::GridMap &);
  bool checkValid(const std::array<int, 2> &);
  bool checkSame(const std::shared_ptr<Node> &, const std::shared_ptr<Node> &);
  std::vector<geometry_msgs::msg::Point>
  getShortestPath(const std::vector<std::shared_ptr<Node>> &);
};

#endif // A_STAR__A_STAR_HPP_
