#include <chrono>
#include <memory>
#include <vector>

#include "costmap_node.hpp"
#include <math.h>

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
    // Initialize the constructs and their parameters
    string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}

std::vector<int8_t> CostmapNode::flatternOccupancyGrid()
{
    std::vector<int8_t> flatterned;
    for (int i = 0; i < SIZE_OF_MAP; ++i)
    {
        for (int j = 0; j < SIZE_OF_MAP; ++j)
        {
            flatterned.push_back(occupancyGrid[i][j]);
        }
    }
    return flatterned;
}

void CostmapNode::publishCostmap()
{
    auto message = nav_msgs::msg::OccupancyGrid();
    message.header.frame_id = "costmap";
    message.info.resolution = 0.1;
    message.info.width = SIZE_OF_MAP;
    message.info.height = SIZE_OF_MAP;
    message.info.origin.position.x = 0;
    message.info.origin.position.y = 0;
    message.data = flatternOccupancyGrid();

    RCLCPP_INFO(this->get_logger(), "Publishing costmap");

    costmap_pub_->publish(message);
    printOccupancyGrid();
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2!";

    auto message2 = std_msgs::msg::String();
    message2.data = "AwawawaawawA";

    // auto message3 = nav_msgs::msg::OccupancyGrid();

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s' %s", message.data.c_str(), message2.data.c_str());

    string_pub_->publish(message);
    string_pub_->publish(message2);
}

// this func is called whenever lidar sub recieves msg
void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    RCLCPP_INFO(this->get_logger(), "Received lidar data:");

    if (!scan)
    {
        RCLCPP_ERROR(this->get_logger(), "error: no lidar data");
        return;
    }

    initializeCostmap(SIZE_OF_MAP); // 200 cells l * w

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range < scan->range_max && range > scan->range_min)
        {                                                                          // within grid
            std::pair<int, int> point = getGridIndicies({100, 100}, range, angle); // set origin as center (100,100)
            int x_grid = point.first;
            int y_grid = point.second;
            // convertToGrid(range, range, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }

        // Log the angle
        // RCLCPP_INFO(this->get_logger(), "Angle: %.2f radians", angle);
    }
    // Step 3: Inflate obstacles
    inflateObstacles(INFLATION_RADIUS);

    // Step 4: Publish costmap
    publishCostmap();
    // printOccupancyGrid();
}

void CostmapNode::initializeCostmap(int size)
{                                                          // 0.1m per cell, 200 cells, 20m
    occupancyGrid.resize(size, std::vector<int>(size, 0)); // set array size

    // init all cells as 0 (empty)
    for (int i = 0; i < size; ++i)
    {
        for (int j = 0; j < size; ++j)
        {
            occupancyGrid[i][j] = 0;
        }
    }
}

void CostmapNode::printOccupancyGrid()
{
    for (int i = 0; i < SIZE_OF_MAP; ++i)
    {
        std::string row_str;
        for (int j = 0; j < SIZE_OF_MAP; ++j)
        {
            row_str += std::to_string(occupancyGrid[i][j]) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Row %d: %s", i, row_str.c_str());
    }
}

std::pair<int, int> CostmapNode::getGridIndicies(std::pair<int, int> origin, double range, double angle)
{
    const int xDiff = range * cos(angle) / RESOLUTION;
    const int yDiff = range * sin(angle) / RESOLUTION;
    const int x = xDiff + origin.first;
    const int y = yDiff + origin.second;
    int protectedX = (x > SIZE_OF_MAP) ? SIZE_OF_MAP : x;
    int protectedY = (y > SIZE_OF_MAP) ? SIZE_OF_MAP : y;
    protectedX = (protectedX < 0) ? 0 : protectedX;
    protectedY = (protectedY < 0) ? 0 : protectedY;
    return {protectedX, protectedY};
}

void CostmapNode::markObstacle(int x_grid, int y_grid)
{
    if (x_grid >= 0 && x_grid < SIZE_OF_MAP &&
        y_grid >= 0 && y_grid < SIZE_OF_MAP)
    {
        occupancyGrid[x_grid][y_grid] = 100;
    }
}

void CostmapNode::inflateObstacles(int radius)
{
    for (int y = 0; y < SIZE_OF_MAP; ++y)
    {
        for (int x = 0; x < SIZE_OF_MAP; ++x)
        {
            for (int i = -radius; i <= radius; ++i)
            {
                for (int j = -radius; j <= radius; ++j)
                {
                    if (x + i >= 0 && x + i < SIZE_OF_MAP && y + j >= 0 && y + j < SIZE_OF_MAP)
                    {
                        const double distance = sqrt(i * i + j * j);
                        const double cost = MAX_COST * (1 - distance / radius);
                        if (distance <= radius && occupancyGrid[x + i][y + j] < cost &&
                            occupancyGrid[x][y] == 100)
                        {
                            occupancyGrid[x + i][y + j] = cost;
                        }
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}