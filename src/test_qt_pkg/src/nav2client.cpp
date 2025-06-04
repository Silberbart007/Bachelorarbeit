#include "nav2client.h"

Nav2Client::Nav2Client(const rclcpp::NodeOptions & options)
: Node("nav2_client", options)
{
    // Erstelle Pose und Waypoint Clients
    m_pose_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    m_path_client = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    // Warte bis der Server bereit ist
    if (!m_pose_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server (pose) nicht erreichbar!");
    }
    if (!m_path_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server (path) nicht erreichbar!");
    }
}

// Send-Funktion für FollowPath client (Callbacks mit Lambda definiert)
bool Nav2Client::sendPath(const nav_msgs::msg::Path & path)
{
    // Prüfe, ob der FollowPath-Client bereit ist
    if (!m_path_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "FollowPath Action Server nicht erreichbar");
        return false;
    }

    FollowPath::Goal goal_msg;
    goal_msg.path = path;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;

    // Goal-Response
    options.goal_response_callback = [this](std::shared_ptr<GoalHandleFollowPath> goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "FollowPath-Goal wurde abgelehnt");
        } else {
            RCLCPP_INFO(this->get_logger(), "FollowPath-Goal akzeptiert");
        }
    };

    // Feedback: Restdistanz
    options.feedback_callback = [this](GoalHandleFollowPath::SharedPtr,
                                       const std::shared_ptr<const FollowPath::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "FollowPath Feedback erhalten");
    };

    // Ergebnis
    options.result_callback = [this](const GoalHandleFollowPath::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Pfad erfolgreich abgefahren!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "FollowPath wurde abgebrochen.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "FollowPath wurde gecancelt.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unbekannter Fehler bei FollowPath.");
                break;
        }
    };

    m_path_client->async_send_goal(goal_msg, options);
    return true;
}


// Send-Funktion für Pose-Client (Callbacks in seperaten Funktionen definiert)
bool Nav2Client::sendGoal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
  if (!m_pose_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server (pose) nicht erreichbar");
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

  auto future_goal_handle = m_pose_client->async_send_goal(goal_msg, send_goal_options);

  return true;
}

// Alle Ziele des Pose Clients canceln
bool Nav2Client::cancelGoalsPose() 
{
  // Auf server warten
  if (!m_pose_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server (pose) nicht erreichbar");
    return false;
  }

  auto cancel_future = m_pose_client->async_cancel_all_goals();

  // Goals canceln
  if (cancel_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Alle Goals wurden erfolgreich abgebrochen.");
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Abbruch der Goals hat zu lange gedauert oder ist fehlgeschlagen.");
    return false;
  }

  return true;
}

// Goal-Callback für Pose-Client
void Nav2Client::goalResponseCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

// Feedback-Callback für Pose-Client
void Nav2Client::feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    auto distance = feedback->distance_remaining;
    RCLCPP_INFO(this->get_logger(), "Restdistanz: %.2f m", distance);
}

// Result-Callback für Pose-Client
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
