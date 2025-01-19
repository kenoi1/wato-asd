#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "planner_core.hpp"

#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include "a_star_struct.hpp"

enum class State
{
  WAITING_FOR_GOAL,
  WAITING_FOR_ROBOT_TO_REACH_GOAL
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  bool goalReached();
  void planPath();

private:
  robot::PlannerCore planner_;
  State state_ = State::WAITING_FOR_GOAL;
  // Subscribers and Publisher
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data Storage
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose robot_pose_;

  bool goal_received_ = false;
  const std::string FRAME_ID = "sim_world";

  CellIndex generateMap(const geometry_msgs::msg::Point &point);  // world to grid
  geometry_msgs::msg::Point generateWorld(const CellIndex &cell); // grid to world
  double calculateDist(const CellIndex &start, const CellIndex &goal);
  std::vector<CellIndex> getNeighbors(const CellIndex &current_node);
  bool isValidCell(const CellIndex &cell);
  std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(const AStarNode &goal_node);
  int OBSTACLE_THRESHOLD = 15;
  // top of queue is best node
  std::unordered_map<CellIndex, AStarNode, CellIndexHash> node_map_;
};

#endif
