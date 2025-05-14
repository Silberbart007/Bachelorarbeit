#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <functional>


class RobotNode : public rclcpp::Node {
public:
    RobotNode();

    // Publish Funktionen
    void publish_velocity(double speed, double rotation);

    // Getter
    double getVelocity()    { return m_vel; };
    double getRotation()    { return m_rot; };

    // GUI-Callback-Funktionen
    std::function<void(const sensor_msgs::msg::Image::SharedPtr&)> on_image_received;
    std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr&)> on_scan_received;

private:
    // Robot attributes
    double m_vel;
    double m_rot;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmd_pub;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;

    // Callback
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};
