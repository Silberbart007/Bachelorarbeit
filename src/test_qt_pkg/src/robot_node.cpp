#include "robot_node.h"
#include <chrono>

RobotNode::RobotNode() : Node("robot_node")
{
    //Eigenschaften des Roboters setzen
    m_speed.x = 0.0;
    m_speed.y = 0.0;
    m_rot = 0.0;
    m_max_speed.x = 25.0;
    m_max_speed.y = 25.0;
    m_max_rotation = 1.5;

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

void RobotNode::publish_velocity(RobotSpeed speed, double rotation)
{
    // Message vorbereiten
    auto msg = geometry_msgs::msg::Twist();
    m_speed.x = speed.x * m_max_speed.x;
    m_speed.y = speed.y * m_max_speed.y;
    m_rot = rotation * m_max_rotation;

    // Variablen des Roboters setzen
    msg.linear.x = m_speed.x;
    msg.linear.y = m_speed.y;
    msg.angular.z = m_rot;

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


// Normierte Speed [-1.0 bis 1.0] kriegen
RobotNode::RobotSpeed RobotNode::getSpeedNormalized() {
    RobotSpeed normSpeed;

    normSpeed.x = (m_max_speed.x != 0.0) ? (m_speed.x / m_max_speed.x) : 0.0;
    normSpeed.y = (m_max_speed.y != 0.0) ? (m_speed.y / m_max_speed.y) : 0.0;

    return normSpeed;
}

// Normierte Rotation [-1.0 bis 1.0] kriegen
double RobotNode::getRotationNormalized() {
    // Rotation in [-max_rotation .. max_rotation] auf [-1..1] normieren
    if (m_max_rotation == 0) return 0.0;
    return m_rot / m_max_rotation;
}

void start_robot_node() {
  rclcpp::spin(std::make_shared<RobotNode>());
}
