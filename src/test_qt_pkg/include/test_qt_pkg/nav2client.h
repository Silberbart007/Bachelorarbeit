#ifndef NAV2_CLIENT_H
#define NAV2_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <functional>

class Nav2Client : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit Nav2Client(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    bool sendGoal(const geometry_msgs::msg::PoseStamped & goal_pose);

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

    void goalResponseCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
};

#endif  // NAV2_CLIENT_H
