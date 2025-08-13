/**
 * @file nav2_client.h
 * @brief Header for Nav2Client, a class that interfaces with the Nav2 action servers for navigation
 * and path following.
 *
 * This class encapsulates interaction with the ROS 2 Nav2 stack, providing methods
 * for sending navigation goals and paths to a robot, as well as handling feedback and results.
 *
 * @author Max Vtulkin
 * @date 2025
 */

#ifndef NAV2_CLIENT_H
#define NAV2_CLIENT_H

// ROS 2 core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Nav2 action definitions
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

// Message types
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

// GUI forward declarations
#include "mainwindow.h"
#include "obstacle_map_widget.h"

#include "test_qt_pkg/srv/plan_path.hpp"

#include <functional>

class ObstacleMapWidget;

/**
 * @class Nav2Client
 * @brief ROS 2 Node for interacting with Nav2 action servers.
 *
 * This class provides an interface for sending navigation goals and paths to the Nav2 stack.
 * It handles goal responses, feedback, and results, and can optionally update a connected UI
 * component.
 */
class Nav2Client : public rclcpp::Node {
  public:
    /// Type aliases for NavigateToPose action
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /// Type aliases for FollowPath action
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    /**
     * @brief Constructor for Nav2Client.
     * @param options Optional node options for parameter/configuration setup.
     */
    explicit Nav2Client(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Sends a goal pose to the NavigateToPose action server.
     * @param goal_pose The target pose for the robot to navigate to.
     * @return True if the goal was accepted and is being processed.
     */
    bool sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose);

    /**
     * @brief Sends a path to the FollowPath action server.
     * @param path A nav_msgs::Path that the robot should follow.
     * @return True if the path was accepted and is being processed.
     */
    bool sendPath(const nav_msgs::msg::Path& path);

    /**
     * @brief Attempts to cancel the current goal sent to the NavigateToPose server.
     * @return True if a cancellation request was made (not necessarily successful).
     */
    bool cancelGoalsPose();

    /**
     * @brief Attempts to cancel the current goal sent to the FollowPath server.
     * @return True if a cancellation request was made (not necessarily successful).
     */
    bool cancelGoalsFollow();

    /**
     * @brief Sets the associated obstacle map widget to be updated with feedback/status.
     * @param obstacle_map Pointer to the ObstacleMapWidget UI element.
     */
    void setObstacleMap(ObstacleMapWidget* obstacle_map) {
        m_obstacle_map = obstacle_map;
    };

    /**
     * @brief Set FollowPath Finish Callback function
     * @param cb Target function to be set
     */
    void setOnPathFinishedCallback(std::function<void()> cb) {
        onPathFinishedCallback = cb;
    }

  private:
    /// Pointer to the obstacle map widget for UI integration (can be nullptr).
    ObstacleMapWidget* m_obstacle_map;

    /// Action client for NavigateToPose
    rclcpp_action::Client<NavigateToPose>::SharedPtr m_pose_client;

    /// Action client for FollowPath
    rclcpp_action::Client<FollowPath>::SharedPtr m_path_client;

    /// Handle to the currently active NavigateToPose goal, if any
    rclcpp_action::Client<NavigateToPose>::GoalHandle::SharedPtr m_current_goal_handle;

    /**
     * @brief Callback triggered when the goal is either accepted or rejected by the server.
     * @param goal_handle The goal handle provided by the server.
     */
    void goalResponseCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

    /**
     * @brief Callback for intermediate feedback from the NavigateToPose action.
     * @param goal_handle The handle of the goal receiving feedback.
     * @param feedback Feedback information (e.g. current robot position).
     */
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr goal_handle,
                          const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    /**
     * *@brief Callback for feedback on finishing followpath
     */
    std::function<void()> onPathFinishedCallback;

    /**
     * @brief Callback for the final result of the navigation goal.
     * @param result Wrapped result of the navigation goal (success, failure, etc.).
     */
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result);
};

#endif // NAV2_CLIENT_H
