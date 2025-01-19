#include "planner_node.hpp"
// #include "a_star_struct.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL)
{
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached()
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 1.5; // Threshold for reaching the goal
}

void PlannerNode::planPath()
{
  if (!goal_received_ || current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set; // nodes to process
  std::unordered_map<CellIndex, bool, CellIndexHash> closed_set; // visited map

  node_map_.clear();

  // A* Implementation (pseudo-code)
  /*
 - Open set: Sets to be evaluated
 - closed set: sets already evaluated
 - f = g+h
 - UNIT FOR G: ADJ +10, DIAG +14
 */

  CellIndex start = generateMap(robot_pose_.position);
  CellIndex goal = generateMap(goal_.point);

  RCLCPP_INFO(this->get_logger(), "Planning path from (%d, %d) to (%d, %d)",
              start.x, start.y, goal.x, goal.y);
  RCLCPP_INFO(this->get_logger(), "Robot position: (%f, %f), Goal position: (%f, %f)",
              robot_pose_.position.x, robot_pose_.position.y,
              goal_.point.x, goal_.point.y);
  RCLCPP_INFO(this->get_logger(), "Map info: width=%d, height=%d, resolution=%f",
              current_map_.info.width, current_map_.info.height,
              current_map_.info.resolution);

  // init start node
  double h_score = calculateDist(start, goal);
  AStarNode starting_node(
      start,
      0.0 + h_score,
      0.0,
      h_score,
      start);

  // at start, its the only one eval.
  open_set.push(starting_node);
  node_map_[start] = starting_node;

  while (!open_set.empty()) 
  {
    AStarNode current_node = open_set.top(); // add node to visiting
    open_set.pop();                          // remove
    
    closed_set[current_node.index] = true;   // set as visited
    
    // RCLCPP_INFO(this->get_logger(), "my loop worked");

    // TODO
    if (current_node.index == goal)
    { // yay!
      RCLCPP_INFO(this->get_logger(), "THIS IS THE END");
      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = FRAME_ID;
      path.poses = reconstructPath(current_node);
      path_pub_->publish(path);
      return;
    }

    for (const auto &neighbor_index : getNeighbors(current_node.index))
    {
      if (closed_set.find(neighbor_index) != closed_set.end()) {
        // Already visited
        continue;
      }

      double tentative_g;
      if (abs(current_node.index.x - neighbor_index.x) == 1 &&
          abs(current_node.index.y - neighbor_index.y) == 1)
      {
        // Cell is diagonal to the existing cell
        tentative_g = current_node.g_score + std::sqrt(2.0);
      }
      else
      {
        tentative_g = current_node.g_score + 1.0; // orthogonal
      }

      // check potentially better route
      if (node_map_.find(neighbor_index) == node_map_.end() || 
          tentative_g < node_map_[neighbor_index].g_score)
      {
        // update neighbor if found shorter path
        node_map_[neighbor_index] = AStarNode(
            neighbor_index,                                    // index
            tentative_g + calculateDist(neighbor_index, goal), // f_score (g + h)
            tentative_g,                                       // g_score
            calculateDist(neighbor_index, goal),               // h_score
            current_node.index);

        open_set.push(node_map_[neighbor_index]);
      }
    }
  }

  RCLCPP_WARN(this->get_logger(), "No path found!");
//

// Compute path using A* on current_map_
// Fill path.poses with the resulting waypoints.
}
double PlannerNode::calculateDist(const CellIndex &start, const CellIndex &goal)
{
  // Euclidean distance
  double dx = start.x - goal.x;
  double dy = start.y - goal.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex &current_node)
{
  std::vector<CellIndex> neighbors_of_node;
  // get all 8 neighbours of a node in 3 by 3
  for (int dx = -1; dx <= 1; dx++)
  {
    for (int dy = -1; dy <= 1; dy++)
    {
      if (dx == 0 && dy == 0)
        continue; // skip, the node

      CellIndex neighbor(current_node.x + dx, current_node.y + dy);
      if (isValidCell(neighbor))
      { // also check within bounds
        neighbors_of_node.push_back(neighbor);
      }
    }
  }
  return neighbors_of_node;
}

bool PlannerNode::isValidCell(const CellIndex &cell)
{
  // check bounds
  if (cell.x < 0 || cell.x >= current_map_.info.width ||
      cell.y < 0 || cell.y >= current_map_.info.height)
  {
    return false;
  }
  // index in 1d array
  int cell_index = cell.y * current_map_.info.width + cell.x;
  int cell_cost = static_cast<int>(current_map_.data.at(cell_index));
  if (cell_cost >= OBSTACLE_THRESHOLD) {
    return false;
  }
  return true;
}
double PlannerNode::getCellCost
CellIndex PlannerNode::generateMap(const geometry_msgs::msg::Point &point)
{
  // Convert world coordinates (meters) to grid coordinates (cells)
  int x = static_cast<int>((point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int y = static_cast<int>((point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  return CellIndex(x, y);
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerNode::reconstructPath(const AStarNode &goal_node)
{
  std::vector<geometry_msgs::msg::PoseStamped> path;

  // Start from goal node
  AStarNode current = goal_node;

  // Trace back through parents until we reach start (where parent == current)
  while (!(current.index == current.parent))
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = FRAME_ID; // fix frame
    pose.header.stamp = this->get_clock()->now();

    // Convert grid coordinates back to world coordinates
    geometry_msgs::msg::Point point = generateWorld(current.index);
    pose.pose.position = point;

    // Add to front of path (since we're working backwards)
    path.insert(path.begin(), pose);

    // Move to parent
    current = node_map_[current.parent];
  }

  // Add start position
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header.frame_id = FRAME_ID;
  start_pose.header.stamp = this->get_clock()->now();
  start_pose.pose.position = generateWorld(current.index);
  path.insert(path.begin(), start_pose);

  return path;
}

geometry_msgs::msg::Point PlannerNode::generateWorld(const CellIndex &cell)
{
  geometry_msgs::msg::Point point;
  point.x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
  point.y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
  point.z = 0.0;
  return point;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
