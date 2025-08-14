#include "nav2client.h"

// =====================
// Public Methods
// =====================

/**
 * @brief Constructor for the Nav2Client class.
 *
 * Initializes the ROS 2 action clients for `NavigateToPose` and `FollowPath`, and waits
 * for the corresponding Nav2 action servers to become available. If either server is not
 * reachable within 5 seconds, an error is logged.
 *
 * @param options Optional node configuration parameters passed to the base rclcpp::Node.
 */
Nav2Client::Nav2Client(const rclcpp::NodeOptions& options) : Node("nav2_client", options) {
    // Create the action client for NavigateToPose
    m_pose_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Create the action client for FollowPath
    m_path_client = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    m_jakob_client = this->create_client<path_planning_eval::srv::PlanPath>("plan_path");

    // Wait for the NavigateToPose action server to be available
    if (!m_pose_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server (navigate_to_pose) not available!");
    }

    // Wait for the FollowPath action server to be available
    if (!m_path_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server (follow_path) not available!");
    }
}

/**
 * @brief Sends a navigation goal to the NavigateToPose action server.
 *
 * This method constructs a NavigateToPose goal message from the given pose and sends it
 * asynchronously to the action server. It also sets up callbacks for handling the goal response,
 * feedback, and result. The actual handling of these is delegated to separate methods.
 *
 * @param goal_pose The target pose the robot should navigate to.
 * @return true if the goal was sent (regardless of whether the server later accepts it), false if
 * the server was not available.
 */
bool Nav2Client::sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose) {
    // Ensure that the action server is ready before sending the goal
    if (!m_pose_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server (navigate_to_pose) not available");
        return false;
    }

    // Construct the goal message
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    // Set up the goal options with appropriate callbacks
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;

    // Callback for when the server accepts or rejects the goal
    send_goal_options.goal_response_callback =
        [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
            this->goalResponseCallback(goal_handle);
        };

    // Callback for intermediate feedback from the server
    send_goal_options.feedback_callback =
        [this](GoalHandleNavigateToPose::SharedPtr goal_handle,
               const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
            this->feedbackCallback(goal_handle, feedback);
        };

    // Callback for the final result
    send_goal_options.result_callback =
        [this](const GoalHandleNavigateToPose::WrappedResult& result) {
            this->resultCallback(result);
        };

    // Send the goal asynchronously
    auto future_goal_handle = m_pose_client->async_send_goal(goal_msg, send_goal_options);

    // Return true to indicate that the goal was dispatched
    return true;
}

/**
 * @brief Sends a path to the FollowPath action server for execution.
 *
 * This function sends a nav_msgs::msg::Path message to the Nav2 FollowPath action server.
 * It sets inline lambda callbacks for handling goal response, feedback, and result.
 * Upon successful execution, any visual path drawings in the obstacle map are cleared.
 *
 * @param path The path the robot should follow.
 * @return true if the goal was sent successfully, false if the action server was not available.
 */
bool Nav2Client::sendPath(const nav_msgs::msg::Path& path) {
    // Check if the FollowPath action server is ready
    if (!m_path_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available");
        return false;
    }

    // Create the goal message
    FollowPath::Goal goal_msg;
    goal_msg.path = path;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;

    // Callback when the goal is accepted or rejected
    options.goal_response_callback = [this](std::shared_ptr<GoalHandleFollowPath> goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "FollowPath goal was rejected");
        } else {
            RCLCPP_INFO(this->get_logger(), "FollowPath goal accepted");
        }
    };

    // Feedback callback: could use remaining distance or progress info
    options.feedback_callback = [this](GoalHandleFollowPath::SharedPtr,
                                       const std::shared_ptr<const FollowPath::Feedback> feedback) {
        (void)feedback; // Placeholder: feedback not yet used
        RCLCPP_INFO(this->get_logger(), "Received FollowPath feedback");
    };

    // Result callback: handle final outcome
    options.result_callback = [this](const GoalHandleFollowPath::WrappedResult& result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Path successfully followed!");
            if (onPathFinishedCallback) {
                onPathFinishedCallback();
            }
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "FollowPath was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "FollowPath was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown error during FollowPath execution");
            break;
        }
    };

    // Send the goal asynchronously
    m_path_client->async_send_goal(goal_msg, options);
    return true;
}

/**
 * @brief Cancels all currently active goals sent to the NavigateToPose action server.
 *
 * This function attempts to cancel all outstanding or currently executing goals for
 * the NavigateToPose action client. It waits up to 2 seconds for the cancellation to be
 * acknowledged.
 *
 * @return true if the goals were successfully cancelled, false if the server was unavailable or the
 * cancellation timed out.
 */
bool Nav2Client::cancelGoalsPose() {
    // Ensure the action server is available before sending a cancel request
    if (!m_pose_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server (navigate_to_pose) not available");
        return false;
    }

    // Request cancellation of all current goals
    auto cancel_future = m_pose_client->async_cancel_all_goals();

    // Wait up to 2 seconds for the cancellation to complete
    if (cancel_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "All NavigateToPose goals successfully cancelled");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cancelling goals took too long or failed");
        return false;
    }
}

/**
 * @brief Cancels all currently active goals sent to the FollowPath action server.
 *
 * This function attempts to cancel all outstanding or currently executing goals for
 * the FollowPath action client. It waits up to 2 seconds for the cancellation to be
 * acknowledged.
 *
 * @return true if the goals were successfully cancelled, false if the server was unavailable or the
 * cancellation timed out.
 */
bool Nav2Client::cancelGoalsFollow() {
    // Ensure the action server is available before sending a cancel request
    if (!m_path_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server (FollowPath) not available");
        return false;
    }

    // Request cancellation of all current goals
    auto cancel_future = m_path_client->async_cancel_all_goals();

    // Wait up to 2 seconds for the cancellation to complete
    if (cancel_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "All FollowPath goals successfully cancelled");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cancelling goals took too long or failed");
        return false;
    }
}

// =====================
// Private Methods
// =====================

/**
 * @brief Callback invoked when the NavigateToPose action server responds to a goal.
 *
 * This function is called after sending a goal to the NavigateToPose action server.
 * It checks whether the goal was accepted or rejected by the server.
 *
 * @param goal_handle A handle to the goal, or nullptr if the goal was rejected.
 */
void Nav2Client::goalResponseCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
    // Check whether the goal was accepted
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the NavigateToPose server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
}

/**
 * @brief Callback that receives continuous feedback from the NavigateToPose action server.
 *
 * This function is called periodically while the goal is being executed.
 * It logs the remaining distance to the target pose, as reported by the server.
 *
 * @param goal_handle Currently unused handle to the goal.
 * @param feedback Shared pointer to the feedback message containing the remaining distance.
 */
void Nav2Client::feedbackCallback(GoalHandleNavigateToPose::SharedPtr /* goal_handle */,
                                  const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    auto distance = feedback->distance_remaining;
    RCLCPP_INFO(this->get_logger(), "Remaining distance: %.2f m", distance);
}

/**
 * @brief Callback that handles the final result of a NavigateToPose goal.
 *
 * Called once the navigation goal is completed, aborted, or cancelled.
 * Depending on the result, this function logs the outcome and clears the map drawings if
 * successful.
 *
 * @param result Wrapped result containing the outcome status and optional result message.
 */
void Nav2Client::resultCallback(const GoalHandleNavigateToPose::WrappedResult& result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
        if (onPathFinishedCallback) {
            onPathFinishedCallback();
        }
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation aborted!");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Navigation cancelled!");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown navigation result");
        break;
    }
}

void Nav2Client::callJakob(geometry_msgs::msg::PoseStamped& start_pose,
                           geometry_msgs::msg::PoseStamped& target_pose) {
    // if (!m_jakob_client->wait_for_service(std::chrono::seconds(5))) {
    //     qWarning() << "PlanPath Service nicht verfügbar.";
    //     return;
    // }

    // Seed einmalig setzen (z. B. im Konstruktor oder beim ersten Aufruf)
    static bool seeded = false;
    if (!seeded) {
        std::srand(std::time(nullptr));
        seeded = true;
    }

    // Einfachen Zufallswert erzeugen
    int32_t random_seed = std::rand();
    qDebug() << "Random Seed: " << random_seed;

    // Request erstellen
    auto request = std::make_shared<path_planning_eval::srv::PlanPath::Request>();

    request->start = start_pose.pose;
    request->goal = target_pose.pose;

    // Beispielwerte – kannst du dynamisch setzen
    request->algo_nr = 1;
    request->rng_seed = random_seed;

    // Asynchronen Request senden
    auto future = m_jakob_client->async_send_request(
        request, [this](rclcpp::Client<path_planning_eval::srv::PlanPath>::SharedFuture response) {
            qDebug() << "Antwort vom Jakob-Service erhalten.";

            auto result = response.get();
            const auto& plan = result->plan;

            // Beispielausgabe
            qDebug() << "Pfad mit" << plan.path.size() << "Posen empfangen.";

            m_obstacle_map->setJakobPoses(plan.path);
        });
}
