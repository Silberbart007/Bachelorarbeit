#include "../include/test_qt_pkg/robot_node.h"
#include <chrono>

RobotNode::RobotNode() : Node("robot_node")
{
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&RobotNode::scan_callback, this, std::placeholders::_1)
    );

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        std::bind(&RobotNode::image_callback, this, std::placeholders::_1)
    );
}

void RobotNode::publish_velocity(double speed, double rotation)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = speed;
    msg.angular.z = rotation;
    cmd_pub_->publish(msg);
}

void RobotNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Führe zugewiesene Funktion von MainWindow aus
    if (on_scan_received)
        on_scan_received(msg);
    else 
        RCLCPP_INFO(this->get_logger(), "No laserscan function");
}

void RobotNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Führe zugewiesene Funktion von MainWindow aus
    if (on_image_received)
        on_image_received(msg);
    else 
        RCLCPP_INFO(this->get_logger(), "No cam-image function");
}
