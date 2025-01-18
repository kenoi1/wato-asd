#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/odometry.hpp>

class MapMemoryNode : public rclcpp::Node
{
public:
    MapMemoryNode();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    void integrateCostmap();
    void publishMap();

private:
    robot::MapMemoryCore map_memory_;

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr double RESOLUTION = 0.3;
    static constexpr double SIZE_OF_MAP = 100;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x, last_y, last_yaw;
    const double distance_threshold;

    // Flags
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
    bool map_just_init_ = false;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
};

#endif
