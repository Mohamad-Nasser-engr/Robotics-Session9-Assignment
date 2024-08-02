#include "rclcpp/rclcpp.hpp"
//#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/srv/find_closest_wall.hpp"

class WallFollowerNode : public rclcpp::Node{
    public:
    WallFollowerNode();

    private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Client<custom_interfaces::srv::FindClosestWall>::SharedPtr find_closest_wall_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    geometry_msgs::msg::Twist twist_;
    float forward_distance_;
    float left_distance_;
    float right_distance_;
    float target_distance_;
};