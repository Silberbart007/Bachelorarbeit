#include "../include/test_qt_pkg/robot_node.h"
#include <chrono>

RobotNode::RobotNode() : Node("robot_node")
{
    //Eigenschaften des Roboters setzen
    m_vel = 0.0;
    m_rot = 0.0;

    // cmd_vel Publisher
    m_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Scan subscriber
    m_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&RobotNode::scan_callback, this, std::placeholders::_1)
    );

    // Kamera Subscriber
    m_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        std::bind(&RobotNode::image_callback, this, std::placeholders::_1)
    );
}

void RobotNode::publish_velocity(double speed, double rotation)
{
    // Message vorbereiten
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = speed;
    msg.angular.z = rotation;

    // Variablen des Roboters setzen
    m_vel = speed;
    m_rot = rotation;

    // Daten an topic publishen
    m_cmd_pub->publish(msg);
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
