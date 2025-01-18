#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

double quaternionToYaw(double x, double y, double z, double w)
{
    // Compute yaw (rotation around Z-axis)
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger())),
      last_x(0.0),
      last_y(0.0),
      distance_threshold(5.0)
{
    // init sub/pub
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    
    // init
    global_map_.data = {};
    global_map_.data.resize(SIZE_OF_MAP * SIZE_OF_MAP, 0);
    map_just_init_ = true;
    

    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::publishMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double q_x = msg->pose.pose.orientation.x;
    double q_y = msg->pose.pose.orientation.y;
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    double yaw = quaternionToYaw(q_x, q_y, q_z, q_w);

    // update map after 5m dist.
    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold || map_just_init_)
    {
        last_x = x;
        last_y = y;
        last_yaw = yaw;
        map_just_init_ = false;
        should_update_map_ = true;
        updateMap();
    }
}

void MapMemoryNode::publishMap() {
    global_map_.header.stamp = this->now();
    global_map_.header.frame_id = "sim_world";
    map_pub_->publish(global_map_);
}

void MapMemoryNode::updateMap()
{
    if (should_update_map_ && costmap_updated_)
    {
        integrateCostmap();
        global_map_.info.resolution = RESOLUTION;
        global_map_.info.width = SIZE_OF_MAP;
        global_map_.info.height = SIZE_OF_MAP;
        global_map_.info.origin.position.x = -1 * SIZE_OF_MAP / 2 * RESOLUTION;
        global_map_.info.origin.position.y = -1 * SIZE_OF_MAP / 2 * RESOLUTION;

        should_update_map_ = false;
    }
}

void MapMemoryNode::integrateCostmap()
{
    // Implementation of costmap integration
    const std::vector<int8_t> &costmap_data = latest_costmap_.data;
    std::vector<int8_t> &global_map_data = global_map_.data;
    double cos_yaw = std::cos(last_yaw);
    double sin_yaw = std::sin(last_yaw);
    double x_pos = last_x;
    double y_pos = last_y;
    int local_size = latest_costmap_.info.width;
    int local_origin_x = latest_costmap_.info.origin.position.x;
    int local_origin_y = latest_costmap_.info.origin.position.y;
    double local_resolution = latest_costmap_.info.resolution;
    for (uint i = 0; i < local_size; ++i)
    {
        for (uint j = 0; j < local_size; ++j)
        {
            int occupancy_value = costmap_data[i * local_size + j];
            if (occupancy_value < 0) { // unknown
                continue;
            }
            double local_x = local_origin_x + (j+0.5) * local_resolution;
            double local_y = local_origin_y + (i+0.5) * local_resolution;
            double cos_yaw = std::cos(last_yaw);
            double sin_yaw = std::sin(last_yaw);
            double global_x = last_x + (cos_yaw * local_x - sin_yaw * local_y);
            double global_y = last_y + (sin_yaw * local_x + cos_yaw * local_y);
            double origin = SIZE_OF_MAP / -2 * RESOLUTION;
            int map_x = (global_x - origin) / RESOLUTION;
            int map_y = (global_y - origin) / RESOLUTION;
            if (map_x >= 0 && map_x < SIZE_OF_MAP && map_y >= 0 && map_y < SIZE_OF_MAP)
            {
                int map_index = map_y * SIZE_OF_MAP + map_x;
                int8_t &global_occupancy = global_map_data[map_index];
                int cg = (global_occupancy < 0) ? 0 : global_occupancy;
                int merged = std::max(cg, occupancy_value);
                global_occupancy = merged;
            }
        }
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
