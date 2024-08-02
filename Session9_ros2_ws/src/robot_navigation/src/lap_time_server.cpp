#include "lap_time_server.h"

LapTimeServer::LapTimeServer()
: Node("lap_time_server"), leaved_starting_position_(false), near_wall_(false), wall_distance_threshold_(0.3)
{
    action_server_ = rclcpp_action::create_server<MeasureLapTime>(
        this,
        "measure_lap_time",
        std::bind(&LapTimeServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&LapTimeServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&LapTimeServer::handle_accepted, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&LapTimeServer::odom_callback, this, std::placeholders::_1));

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&LapTimeServer::laser_callback, this, std::placeholders::_1));
}

void LapTimeServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_position_x_ = msg->pose.pose.position.x;
    current_position_y_ = msg->pose.pose.position.y;
    current_time_ = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
}

void LapTimeServer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    near_wall_ = std::any_of(msg->ranges.begin(), msg->ranges.end(), [this](float distance) {
        return distance < wall_distance_threshold_;
    });
}

rclcpp_action::GoalResponse LapTimeServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MeasureLapTime::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received request to measure lap time.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LapTimeServer::handle_cancel(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LapTimeServer::handle_accepted(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&LapTimeServer::execute_callback, this, _1), goal_handle}.detach();
}

void LapTimeServer::execute_callback(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal to measure lap time.");
    auto result = std::make_shared<MeasureLapTime::Result>();

    rclcpp::Rate rate(10);  // 10 Hz

    while (!near_wall_)
    {
        rate.sleep();
    }

    while (!start_position_x_ || !start_position_y_)
    {
        start_position_x_ = current_position_x_;
        start_position_y_ = current_position_y_;
        start_time_ = current_time_;
        rate.sleep();
    }

    leaved_starting_position_ = false;
    auto feedback_msg = std::make_shared<MeasureLapTime::Feedback>();

    while (!leaved_starting_position_)
    {
        double distance_from_start = std::sqrt(
            std::pow(current_position_x_ - start_position_x_, 2) +
            std::pow(current_position_y_ - start_position_y_, 2));

        if (distance_from_start > 0.1)
        {
            leaved_starting_position_ = true;
        }
        rate.sleep();
    }

    while (true)
    {
        feedback_msg->elapsed_time = current_time_ - start_time_;
        goal_handle->publish_feedback(feedback_msg);

        double distance_from_start = std::sqrt(
            std::pow(current_position_x_ - start_position_x_, 2) +
            std::pow(current_position_y_ - start_position_y_, 2));

        if (distance_from_start < 0.1)
        {
            break;
        }
        rate.sleep();
    }

    result->total_time = current_time_ - start_time_;
    goal_handle->succeed(result);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LapTimeServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
