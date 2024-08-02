#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/action/measure_lap_time.hpp"

class LapTimeServer : public rclcpp::Node{
    public:
    using MeasureLapTime = custom_interfaces::action::MeasureLapTime;
    using GoalHandleMeasureLapTime = rclcpp_action::ServerGoalHandle<MeasureLapTime>;

    LapTimeServer();

    private:
    rclcpp_action::Server<MeasureLapTime>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    double start_position_x_;
    double start_position_y_;
    double current_position_x_;
    double current_position_y_;
    double start_time_;
    double current_time_;
    bool leaved_starting_position_;
    bool near_wall_;
    const double wall_distance_threshold_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MeasureLapTime::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    void execute_callback(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);

};