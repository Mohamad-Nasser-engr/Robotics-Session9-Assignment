#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/find_closest_wall.hpp" 
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class WallFinderServer : public rclcpp::Node{
    public:
        WallFinderServer();

    private:
        void handle_service(const std::shared_ptr<custom_interfaces::srv::FindClosestWall::Request> request,
        std::shared_ptr<custom_interfaces::srv::FindClosestWall::Response> response);

        void laser_scan_callback(const sensor_msgs::msg::LaserScan msg);

        rclcpp::Service<custom_interfaces::srv::FindClosestWall>::SharedPtr service_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;

        float distance;
        bool service_called;
};