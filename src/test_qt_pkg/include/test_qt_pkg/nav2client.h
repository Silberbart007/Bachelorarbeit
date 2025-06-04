#ifndef NAV2_CLIENT_H
#define NAV2_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <functional>

class Nav2Client : public rclcpp::Node
{
public:
    // Abkürzungen für Navigate to Pose - Namen
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // Abkürzungen für Waypoint-Follow-Namen
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    explicit Nav2Client(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    bool sendGoal(const geometry_msgs::msg::PoseStamped & goal_pose);
    bool sendPath(const nav_msgs::msg::Path & path);
    bool cancelGoalsPose(); 

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr m_pose_client;
    rclcpp_action::Client<FollowPath>::SharedPtr m_path_client;

    rclcpp_action::Client<NavigateToPose>::GoalHandle::SharedPtr m_current_goal_handle;

    void goalResponseCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
};

#endif  // NAV2_CLIENT_H
