#include "nav2client.h"

Nav2Client::Nav2Client(const rclcpp::NodeOptions & options)
: Node("nav2_client", options)
{
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Warte bis der Server bereit ist
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server nicht erreichbar!");
    }
}

bool Nav2Client::sendGoal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
  if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server nicht erreichbar");
    return false;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;

  send_goal_options.goal_response_callback =
  [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      this->goalResponseCallback(goal_handle);
  };

  send_goal_options.feedback_callback = [this](GoalHandleNavigateToPose::SharedPtr goal_handle,
                                               const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    this->feedbackCallback(goal_handle, feedback);
  };

  send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
    this->resultCallback(result);
  };

  auto future_goal_handle = client_->async_send_goal(goal_msg, send_goal_options);

  return true;
}


void Nav2Client::goalResponseCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}


void Nav2Client::feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    auto distance = feedback->distance_remaining;
    RCLCPP_INFO(this->get_logger(), "Restdistanz: %.2f m", distance);
}

void Nav2Client::resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation erfolgreich!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation abgebrochen!");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation abgebrochen (cancelled)!");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unbekanntes Ergebnis der Navigation");
            break;
    }
}
