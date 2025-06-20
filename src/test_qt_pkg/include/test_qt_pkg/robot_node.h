/**
 * @file robot_node.hpp
 * @brief Defines the RobotNode class for handling ROS2 communication with a mobile robot.
 *
 * This class encapsulates the ROS publishers and subscribers necessary to:
 * - Publish velocity commands to the robot
 * - Receive camera images and laser scans
 * - Receive and store map data
 * - Handle AMCL localization data
 *
 * The class is designed to be easily integrated with a Qt-based GUI, exposing
 * callback hooks for sensor and pose data updates.
 *
 * @author Max Vtulkin
 * @date 2025
 */

#pragma once

#include <functional>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @brief ROS2 node managing robot communication: publishing velocity commands and receiving sensor
 * data.
 */
class RobotNode : public rclcpp::Node {
  public:
    /**
     * @brief Construct a new RobotNode instance.
     */
    RobotNode();

    /**
     * @brief Struct representing 2D linear speed.
     */
    struct RobotSpeed {
        double x; ///< Linear velocity along the X axis.
        double y; ///< Linear velocity along the Y axis.
    };

    /**
     * @brief Publish a velocity command to the robot.
     *
     * @param speed Linear velocity in x and y directions.
     * @param rotation Angular velocity around the Z axis.
     */
    void publish_velocity(RobotSpeed speed, double rotation);

    /**
     * @brief Get the current speed (non-normalized).
     * @return Current linear speed as RobotSpeed.
     */
    RobotSpeed getSpeed();

    /**
     * @brief Get the current angular velocity (non-normalized).
     * @return Current angular velocity.
     */
    double getRotation();

    /**
     * @brief Get the current speed normalized to the maximum speed.
     * @return Normalized linear speed.
     */
    RobotSpeed getSpeedNormalized();

    /**
     * @brief Get the current angular velocity normalized to the maximum.
     * @return Normalized angular velocity.
     */
    double getRotationNormalized();

    /**
     * @brief Get the maximum configured linear speed.
     * @return Maximum RobotSpeed.
     */
    RobotSpeed getMaxSpeed();

    /**
     * @brief Get the maximum configured angular velocity.
     * @return Maximum angular velocity.
     */
    double getMaxRotation();

    /**
     * @brief Check if a map has been received from the map topic.
     * @return True if map is available.
     */
    bool has_map() const;

    /**
     * @brief Get the last received map.
     * @return The latest OccupancyGrid map.
     */
    nav_msgs::msg::OccupancyGrid get_map() const;

    /**
     * @brief Callback for receiving camera images.
     */
    std::function<void(const sensor_msgs::msg::Image::SharedPtr&)> on_image_received;

    /**
     * @brief Callback for receiving laser scans.
     */
    std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr&)> on_scan_received;

    /**
     * @brief Callback triggered after the first map is received.
     */
    std::function<void()> map_loaded;

    /**
     * @brief Callback for receiving AMCL pose updates.
     */
    std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr&)>
        on_amcl_pose_received;

  private:
    // --- Robot state ---

    RobotSpeed m_speed;     ///< Current linear speed.
    double m_rot;           ///< Current angular velocity.
    RobotSpeed m_max_speed; ///< Maximum allowed linear speed.
    double m_max_rotation;  ///< Maximum allowed angular velocity.

    // --- Map state ---

    bool m_map_received = false;             ///< Whether a map has been received.
    nav_msgs::msg::OccupancyGrid m_last_map; ///< Last received occupancy grid map.

    // --- ROS Publishers ---

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        m_cmd_pub; ///< Publisher for velocity commands.

    // --- ROS Subscribers ---

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        m_scan_sub; ///< Subscriber for laser scans.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        m_image_sub; ///< Subscriber for camera images.
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        m_map_sub; ///< Subscriber for the occupancy map.
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        m_amcl_sub; ///< Subscriber for AMCL poses.

    // --- Internal Callbacks ---

    /**
     * @brief Internal callback for processing laser scans.
     * @param msg Incoming LaserScan message.
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Internal callback for processing camera images.
     * @param msg Incoming Image message.
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Internal callback for processing occupancy grid maps.
     * @param msg Incoming OccupancyGrid map.
     */
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief Internal callback for processing AMCL pose messages.
     * @param msg Incoming pose with covariance.
     */
    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};
