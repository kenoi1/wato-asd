#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
    // Initialize parameters
    lookahead_distance_ = 1.0; // Lookahead distance
    goal_tolerance_ = 3;       // Distance to consider the goal reached
    linear_speed_ = 0.5;       // Constant forward speed

    // Subscribers and Publishers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg)
        { current_path_ = msg; });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        { robot_odom_ = msg; });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]()
        { controlLoop(); });
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint()
{
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_)
    {
        return std::nullopt; // No valid path or odometry data
    }

    // Extract the robot's current position
    const auto &robot_position = robot_odom_->pose.pose.position;
    const geometry_msgs::msg::PoseStamped &goal = current_path_->poses.back();
    double distance_to_goal = computeDistance(robot_position, goal.pose.position);

    if (distance_to_goal <= goal_tolerance_)
    {
        // If the goal is within the tolerance, stop the robot by returning the robot's current position
        return std::nullopt;
    }

    // Iterate through the path to find the lookahead point
    for (const auto &pose : current_path_->poses)
    {
        const auto &path_point = pose.pose.position;

        // Compute the distance between the robot and the path point
        double distance = computeDistance(robot_position, path_point);

        // Check if the point is at least the lookahead distance away
        if (distance >= lookahead_distance_)
        {
            return pose; // Found a valid lookahead point
        }
    }

    // No lookahead point found
    return std::nullopt;
}

void ControlNode::controlLoop()
{
    // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_)
    {
        return;
    }

    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point)
    {
        return; // No valid lookahead point found
    }

    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);

    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
    // TODO: Implement logic to compute velocity commands
    // Initialize Twist message
    geometry_msgs::msg::Twist velocity;

    // Extract target position
    double target_x = target.pose.position.x;
    double target_y = target.pose.position.y;

    // Extract robot's current position and orientation
    const auto &robot_position = robot_odom_->pose.pose.position;
    const auto &robot_orientation = robot_odom_->pose.pose.orientation;

    // Extract robot's current pose
    double robot_x = robot_position.x;
    double robot_y = robot_position.y;
    double robot_yaw = extractYaw(robot_orientation);
    // Compute distance to the target
    double dx = target_x - robot_x;
    double dy = target_y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Compute the target yaw (angle to the target position)
    double target_yaw = std::atan2(dy, dx);

    // Compute angular velocity to align robot's heading with target
    double angular_velocity = target_yaw - robot_yaw;

    // Normalize angular velocity to the range [-pi, pi]
    while (angular_velocity > M_PI)
        angular_velocity -= 2.0 * M_PI;
    while (angular_velocity < -M_PI)
        angular_velocity += 2.0 * M_PI;

    // Set linear and angular velocities
    velocity.linear.x = distance;          // Distance-based linear velocity (proportional)
    velocity.angular.z = angular_velocity; // Angle-based angular velocity

    return velocity;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat)
{
    // TODO: Implement quaternion to yaw conversion
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);

    // Return yaw angle in radians
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
