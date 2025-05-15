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
    double getSpeed()       { return m_speed; };
    double getRotation()    { return m_rot; };
    double getSpeedNormalized();
    double getRotationNormalized();
    double getMaxSpeed()    { return m_max_speed;};
    double getMaxRotation() { return m_max_rotation;};

    // GUI-Callback-Funktionen
    std::function<void(const sensor_msgs::msg::Image::SharedPtr&)> on_image_received;
    std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr&)> on_scan_received;

private:
    // Robot attributes
    double m_speed;
    double m_rot;
    double m_max_speed;
    double m_max_rotation;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmd_pub;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;

    // Callback
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};
