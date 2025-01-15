#include <chrono>
#include <memory>

#include "costmap_node.hpp"
#include <math.h>
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";

  auto message2 = std_msgs::msg::String();
  message2.data = "AwawawaawawA";

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s' %s", message.data.c_str(), message2.data.c_str());

  string_pub_->publish(message);
  string_pub_->publish(message2);
}
void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  RCLCPP_INFO(this->get_logger(), "Received LaserScan data:");

  
  if (occupancyGrid.empty()) {
        initializeCostmap(SIZE_OF_MAP); // 200 cells l * w
    }

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) { // within grid
      // int x_grid, y_grid;
      // convertToGrid(range, range, x_grid, y_grid);
      // markObstacle(x_grid, y_grid);
    }

    // Log the angle
    // RCLCPP_INFO(this->get_logger(), "Angle: %.2f radians", angle);
  }
}

void CostmapNode::initializeCostmap(int size){ // 0.1m per cell, 200 cells, 20m
  occupancyGrid.resize(size, std::vector<int>(size, 0)); // set size of array
 
 // init all cells as 0 (empty)
  for (int i = 0; i < size; ++i) { 
        for (int j = 0; j < size; ++j) { 
            occupancyGrid[i][j] = 0; 
        } 
    } 
}

void CostmapNode::printOccupancyGrid() {
    for (int i = 0; i < SIZE_OF_MAP; ++i) {
        std::string row_str;
        for (int j = 0; j < SIZE_OF_MAP; ++j) {
            row_str += std::to_string(occupancyGrid[i][j]) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Row %d: %s", i, row_str.c_str());
    }
}

std::pair<int, int> CostmapNode::getGridIndicies(std::pair<int, int> origin, double range, double angle) {
    const int xDiff = range * cos(angle) / RESOLUTION;
    const int yDiff = range * sin(angle) / RESOLUTION;
    const int x = xDiff + origin.first;
    const int y = yDiff + origin.second;
    int protectedX = (x > 20) ? 20 : x;
    int protectedY = (y > 20) ? 20 : x;
    protectedX = (protectedX < 0) ? 0 : protectedX;
    protectedY = (protectedY < 0) ? 0 : protectedY;
    return {protectedX, protectedY};
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}