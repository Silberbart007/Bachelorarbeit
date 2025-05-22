#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <functional>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


class RobotNode : public rclcpp::Node {
public:
    RobotNode();

    struct RobotSpeed
    {
        double x;
        double y;
    };
    

    // Publish Funktionen
    void publish_velocity(RobotSpeed speed, double rotation);

    // Getter
    RobotSpeed getSpeed()   { return m_speed; };
    double getRotation()    { return m_rot; };
    RobotSpeed getSpeedNormalized();
    double getRotationNormalized();
    RobotSpeed getMaxSpeed()    { return m_max_speed;};
    double getMaxRotation() { return m_max_rotation;};

    // Map Funktionen
    bool has_map() const { return m_map_received; }
    nav_msgs::msg::OccupancyGrid get_map() const { return m_last_map; }

    // GUI-Callback-Funktionen
    std::function<void(const sensor_msgs::msg::Image::SharedPtr&)> on_image_received;
    std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr&)> on_scan_received;
    std::function<void()> map_loaded;
    std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr&)> on_amcl_pose_received;

private:
    // Robot attributes
    RobotSpeed m_speed;
    double m_rot;
    RobotSpeed m_max_speed;
    double m_max_rotation;

    // Map Daten
    bool m_map_received = false;
    nav_msgs::msg::OccupancyGrid m_last_map;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmd_pub;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_amcl_sub;

    // Callback
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);   
    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};
