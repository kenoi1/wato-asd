#include "map_memory_node.hpp"
#include <cmath>

double quaternionToYaw(double x, double y, double z, double w) {
    // Compute yaw (rotation around Z-axis)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), last_x(0.0), last_y(0.0), distance_threshold(5.0) {
    // init sub/pub
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double q_x = msg->pose.pose.orientation.x;
    double q_y = msg->pose.pose.orientation.y;
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    double yaw = quaternionToYaw(q_x, q_y, q_z, q_w);

    // update map after 5m dist.
    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        last_yaw = yaw;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
    }
}

void MapMemoryNode::integrateCostmap() {
    // Implementation of costmap integration
    std::vector<int8_t> costmap_data = latest_costmap_.data;
    std::vector<int8_t> global_map_data = global_map_.data;
    double cos_yaw = std::cos(last_yaw);
    double sin_yaw = std::sin(last_yaw);
    double x_pos = last_x / RESOLUTION + 200;
    double y_pos = last_y / RESOLUTION + 200;
    if (global_map_data.empty()) {
        global_map_data.resize(SIZE_OF_MAP * SIZE_OF_MAP, 0);
    }
    for (int local_x = 0; local_x < SIZE_OF_MAP; ++local_x) {
        for (int local_y = 0; local_y < SIZE_OF_MAP; ++local_y) {
            int8_t index = local_x * SIZE_OF_MAP + local_y;
            int8_t shiftedX = cos_yaw * local_x - sin_yaw * local_y + x_pos;
            int8_t shiftedY = sin_yaw * local_x + cos_yaw * local_y + y_pos;
            int8_t shiftedIndex = shiftedX * SIZE_OF_MAP + shiftedY;
            if (shiftedX >= 0 && shiftedX < SIZE_OF_MAP && shiftedY >= 0 && shiftedY < SIZE_OF_MAP &&
                shiftedIndex < global_map_data.size()) {
                global_map_data[shiftedIndex] = costmap_data[index];
            }
        }
    }
    global_map_.data = global_map_data;
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
