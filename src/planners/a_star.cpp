#include "autonomous_motion_planner/planners/a_star.hpp"

AStarSearch::AStarSearch() {}

std::vector<geometry_msgs::msg::Point>
AStarSearch::plan(const geometry_msgs::msg::Point &goal,
                  const grid_map::GridMap &grid_space)
{
  // Store a 2D vector for distances and visisted nodes
  std::vector<std::vector<double>> dist(map_size_x_,
                                        std::vector<double>(map_size_y_, 100000.0));
  std::vector<std::vector<double>> visited(map_size_x_,
                                           std::vector<double>(map_size_y_, false));
  std::vector<std::shared_ptr<Node>> path;

  // Priority que to sort for lowest cost
  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, compareOP>
    priorityQ;

  // Get start index in occupancy grid
  // Start Index will always be 0, 0
  grid_map::Index start_idx;
  grid_map::Position start_pos(0.0, 0.0);
  grid_space.getIndex(start_pos, start_idx);

  // Get goal index in occupancy grid
  grid_map::Index goal_idx;
  grid_map::Position goal_pos(goal.x, goal.y);
  grid_space.getIndex(goal_pos, goal_idx);
  std::shared_ptr<Node> goal_node = std::make_shared<Node>(goal_idx(0), goal_idx(1));

  // Start the search
  std::shared_ptr<Node> first = std::make_shared<Node>(start_idx(0), start_idx(1));
  first->hc = heuristicCost(first, goal_node, grid_space);
  priorityQ.push(first);
  dist[first->x][first->y] = 0.0;

  // Search unitl que is empty
  while(!priorityQ.empty())
  {
    auto curr_node = priorityQ.top();
    priorityQ.pop();
    path.push_back(curr_node);
    visited[curr_node->x][curr_node->y] = true;
    // exit if we found the goal
    if(checkSame(curr_node, goal_node))
    {
      break;
    }
    // Search through neighbors along 8 directions
    // up down left right and 4 diagonals
    for(auto &idx : neighbors)
    {
      std::array<int, 2> neighbor = {curr_node->x + idx[0], curr_node->y + idx[1]};
      if(!visited[neighbor[0]][neighbor[1]] && checkValid(neighbor))
      {
        double curr_dist = curr_node->gc + grid_resolution_;
        if(curr_dist < dist[neighbor[0]][neighbor[1]])
        {
          dist[neighbor[0]][neighbor[1]] = curr_dist;
          std::shared_ptr<Node> adj_node
            = std::make_shared<Node>(neighbor[0], neighbor[1]);
          adj_node->gc = curr_dist;
          adj_node->hc = heuristicCost(adj_node, goal_node, grid_space);
          adj_node->parent = curr_node;
          priorityQ.push(adj_node);
        }
      }
    }
  }
  // END OF ASTAR SEARCH

  // Find the shortest path
  std::vector<geometry_msgs::msg::Point> shortest_path = getShortestPath(path);

  return shortest_path;
}

void AStarSearch::setMapParams(int grid_map_x, int grid_map_y, double grid_resolution,
                               const std::string &layer_name)
{
  map_size_x_ = grid_map_x;
  map_size_y_ = grid_map_y;
  grid_resolution_ = grid_resolution;
  layer_name_ = layer_name;
}

double AStarSearch::heuristicCost(const std::shared_ptr<Node> &node1,
                                  const std::shared_ptr<Node> &node2,
                                  const grid_map::GridMap &grid_space)
{
  double heuristic_cost;

  // Add euclidean heuristics
  double hdistance_cost = sqrt(pow(node1->x - node2->x, 2) + pow(node1->y - node2->y, 2));

  // Add grid cost of the cell
  grid_map::Index node_idx(node1->x, node1->y);
  double hgrid_cost = grid_space.at(layer_name_, node_idx);
  // csafety check for costs that are nan or inf
  if(std::isnan(hgrid_cost))
    hgrid_cost = 0.0;

  if(std::isinf(hgrid_cost))
    hgrid_cost = 1e6;

  heuristic_cost = hdistance_cost + hgrid_cost;

  return heuristic_cost;
}

bool AStarSearch::checkValid(const std::array<int, 2> &node)
{
  if(node[0] < 0 || node[0] > map_size_x_ || node[1] < 0 || node[1] > map_size_y_)
    return false;

  return true;
}

bool AStarSearch::checkSame(const std::shared_ptr<Node> &n1,
                            const std::shared_ptr<Node> &n2)
{
  if(n1->x == n2->x && n1->y == n2->y)
    return true;

  return false;
}

std::vector<geometry_msgs::msg::Point>
AStarSearch::getShortestPath(const std::vector<std::shared_ptr<Node>> &path)
{
  std::vector<geometry_msgs::msg::Point> short_path;
  auto curr_node = path.back();
  while(curr_node != nullptr)
  {
    geometry_msgs::msg::Point pt;
    pt.x = curr_node->x;
    pt.y = curr_node->y;
    pt.z = 0.0;

    short_path.emplace_back(pt);
    curr_node = curr_node->parent;
  }
  // Reverse the list to get the waypoints for start to goal
  std::reverse(short_path.begin(), short_path.end());

  return short_path;
}
