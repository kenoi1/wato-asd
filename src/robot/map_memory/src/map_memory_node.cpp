#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  // init sub/pub
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this));
}

void MappingNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MappingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

// update map after 5m dist.
    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        should_update_map_ = true;
    }
}

void MappingNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
    }
}

void MappingNode::integrateCostmap() {
    // Implementation of costmap integration
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
