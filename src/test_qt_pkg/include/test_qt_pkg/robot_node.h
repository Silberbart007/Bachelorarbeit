#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <functional>


class RobotNode : public rclcpp::Node {
public:
    RobotNode();

    void publish_velocity(double speed, double rotation);

    // GUI-Callback-Funktionen
    std::function<void(const sensor_msgs::msg::Image::SharedPtr&)> on_image_received;
    std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr&)> on_scan_received;

private:
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Callback
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};
