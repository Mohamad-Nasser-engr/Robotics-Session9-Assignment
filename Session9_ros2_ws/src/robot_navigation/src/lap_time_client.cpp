#include "lap_time_client.h"

using MeasureLapTime = custom_interfaces::action::MeasureLapTime;
using GoalHandleMeasureLapTime = rclcpp_action::ClientGoalHandle<MeasureLapTime>;

LapTimeClient::LapTimeClient() : Node("lap_time_client") {  // Changed node name
    this->action_client = rclcpp_action::create_client<MeasureLapTime>(
        this,
        "measure_lap_time"
    );
}

void LapTimeClient::send_goal() {
    using namespace std::placeholders;
    if (!this->action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal_msg = MeasureLapTime::Goal();
    auto send_goal_options = rclcpp_action::Client<MeasureLapTime>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &LapTimeClient::goal_response_callback,
        this,
        _1
    );
    send_goal_options.feedback_callback = std::bind(
        &LapTimeClient::feedback_callback,
        this,
        _1,
        _2
    );
    send_goal_options.result_callback = std::bind(
        &LapTimeClient::result_callback,
        this,
        _1
    );
    this->action_client->async_send_goal(goal_msg, send_goal_options);
}

void LapTimeClient::goal_response_callback(const GoalHandleMeasureLapTime::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal rejected.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted.");
    }
}

void LapTimeClient::feedback_callback(
    GoalHandleMeasureLapTime::SharedPtr,
    const std::shared_ptr<const MeasureLapTime::Feedback> feedback
) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Elapsed lap time: %.3f seconds.", 0.001 * round(1000 * feedback->elapsed_time));

}

void LapTimeClient::result_callback(const GoalHandleMeasureLapTime::WrappedResult & result) {

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal aborted.");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal canceled.");
            return;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result.");
            return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Total lap time: %.3f seconds.", 0.001 * round(1000 * result.result->total_time));
    rclcpp::shutdown();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<LapTimeClient>();
    client_node->send_goal();
    rclcpp::spin(client_node);
    return 0;
}
