#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "costmap_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

    // Place callback function here
    void publishMessage();
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void initializeCostmap(int size);
    void printOccupancyGrid();

private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    static constexpr double RESOLUTION = 0.1;
    static constexpr double SIZE_OF_MAP = 200;
    static constexpr double MAX_COST = 100;
    std::pair<int, int> getGridIndicies(std::pair<int, int> origin, double range, double angle);
    std::vector<std::vector<int>> occupancyGrid;
    void inflate(int x, int y, int radius);
};

#endif