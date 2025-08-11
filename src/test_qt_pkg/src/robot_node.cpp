#include "robot_node.h"
#include <chrono>

// =====================
// Public Methods
// =====================

/**
 * @brief Constructor: Initializes publishers, subscribers, and robot parameters.
 */
RobotNode::RobotNode() : Node("robot_node") {
    // Set initial robot speed and limits
    m_speed.x = 0.0;
    m_speed.y = 0.0;
    m_rot = 0.0;
    m_max_speed.x = 0.2;
    m_max_speed.y = 0.2;
    m_max_rotation = 0.8;

    // Create publisher for velocity commands
    m_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribe to laser scan data
    m_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/base_scan", 10, std::bind(&RobotNode::scan_callback, this, std::placeholders::_1));

    // Subscribe to camera image stream
    m_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10, std::bind(&RobotNode::image_callback, this, std::placeholders::_1));

    // Subscribe to occupancy grid map with reliable and transient QoS
    auto qos = rclcpp::QoS(10);
    qos.transient_local(); // Ensures late joiners still get the last message
    qos.reliable();        // Ensures delivery
    m_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&RobotNode::map_callback, this, std::placeholders::_1));

    // Subscribe to AMCL pose estimate
    m_amcl_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&RobotNode::amcl_callback, this, std::placeholders::_1));

    // Subscribe to cmd
    m_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { m_last_cmd_vel = *msg; });
}

/**
 * @brief Publishes a velocity command to the robot.
 *
 * Multiplies normalized input by max speed/rotation before publishing.
 *
 * @param speed Normalized linear speed (-1.0 to 1.0) in X and Y.
 * @param rotation Normalized angular speed (-1.0 to 1.0).
 */
void RobotNode::publish_velocity(RobotSpeed speed, double rotation) {
    // Prepare Twist message
    auto msg = geometry_msgs::msg::Twist();
    m_speed.x = speed.x * m_max_speed.x;
    m_speed.y = speed.y * m_max_speed.y;
    m_rot = rotation * m_max_rotation;

    msg.linear.x = m_speed.x;
    msg.linear.y = m_speed.y;
    msg.angular.z = -m_rot; // Negated for coordinate system alignment

    // Publish the velocity command
    m_cmd_pub->publish(msg);
}

/**
 * @brief Returns the current unnormalized linear speed.
 * @return The robot's current RobotSpeed.
 */
RobotNode::RobotSpeed RobotNode::getSpeed() {
    return m_speed;
}

/**
 * @brief Returns the current unnormalized angular velocity.
 * @return The robot's current angular velocity.
 */
double RobotNode::getRotation() {
    return m_rot;
}

/**
 * @brief Returns the current speed normalized to [-1.0, 1.0].
 *
 * @return Normalized linear speed.
 */
RobotNode::RobotSpeed RobotNode::getSpeedNormalized() {
    RobotSpeed normSpeed;
    normSpeed.x = (m_max_speed.x != 0.0) ? (m_speed.x / m_max_speed.x) : 0.0;
    normSpeed.y = (m_max_speed.y != 0.0) ? (m_speed.y / m_max_speed.y) : 0.0;
    return normSpeed;
}

/**
 * @brief Returns the current angular speed normalized to [-1.0, 1.0].
 *
 * @return Normalized rotation.
 */
double RobotNode::getRotationNormalized() {
    return (m_max_rotation != 0.0) ? (m_rot / m_max_rotation) : 0.0;
}

/**
 * @brief Returns the configured maximum linear speed.
 * @return The maximum RobotSpeed.
 */
RobotNode::RobotSpeed RobotNode::getMaxSpeed() {
    return m_max_speed;
}

/**
 * @brief Returns the configured maximum angular velocity.
 * @return Maximum angular velocity.
 */
double RobotNode::getMaxRotation() {
    return m_max_rotation;
}

/**
 * @brief Get current cmd Speed
 * @return Robot Speed
 */
geometry_msgs::msg::Twist RobotNode::getLastCmdVel() const {
    return m_last_cmd_vel;
}

/**
 * @brief Returns whether a map has been received.
 * @return True if a map is available.
 */
bool RobotNode::has_map() const {
    return m_map_received;
}

/**
 * @brief Returns the last received occupancy grid map.
 * @return Latest map as nav_msgs::msg::OccupancyGrid.
 */
nav_msgs::msg::OccupancyGrid RobotNode::get_map() const {
    return m_last_map;
}

// =====================
// Private Methods
// =====================

/**
 * @brief Callback for laser scan messages.
 *
 * Passes the message to the GUI if a callback is connected.
 *
 * @param msg LaserScan message received from /base_scan.
 */
void RobotNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (on_scan_received)
        on_scan_received(msg);
    else
        RCLCPP_INFO(this->get_logger(), "No laserscan function connected.");
}

/**
 * @brief Callback for image messages.
 *
 * Passes the image to the GUI if a callback is connected.
 *
 * @param msg Image message received from /camera/image_raw.
 */
void RobotNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (on_image_received)
        on_image_received(msg);
    else
        RCLCPP_INFO(this->get_logger(), "No camera image function connected.");
}

/**
 * @brief Callback for occupancy grid map messages.
 *
 * Stores the latest map and notifies the GUI if connected.
 *
 * @param msg OccupancyGrid map received from /map.
 */
void RobotNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Map Callback triggered.");
    m_last_map = *msg;
    m_map_received = true;
    RCLCPP_INFO(this->get_logger(), "Received map: %d x %d", msg->info.width, msg->info.height);

    if (map_loaded)
        map_loaded();
}

/**
 * @brief Callback for AMCL pose updates.
 *
 * Forwards the data to the GUI if a callback is set.
 *
 * @param msg Pose message with covariance from /amcl_pose.
 */
void RobotNode::amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (on_amcl_pose_received)
        on_amcl_pose_received(msg);
    else
        RCLCPP_INFO(this->get_logger(), "No AMCL pose function connected.");
}

/**
 * @brief Starts spinning the RobotNode. Blocking call.
 */
void start_robot_node() {
    rclcpp::spin(std::make_shared<RobotNode>());
}