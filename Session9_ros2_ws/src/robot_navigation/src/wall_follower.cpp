#include "wall_follower.h"

    WallFollowerNode::WallFollowerNode()
        : Node("wall_follower_node")
    {
        // Create a service client to call FindClosestWall
        find_closest_wall_client_ = this->create_client<custom_interfaces::srv::FindClosestWall>("wall_finder");
        while (!find_closest_wall_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        // Create a publisher for /cmd_vel
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create a subscription to /scan
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollowerNode::scan_callback, this, std::placeholders::_1));

        twist_ = geometry_msgs::msg::Twist();
        forward_distance_ = std::numeric_limits<float>::infinity();
        left_distance_ = std::numeric_limits<float>::infinity();
        right_distance_ = std::numeric_limits<float>::infinity();
        target_distance_ = 0.5; // Target distance to the wall in meters

        auto req = std::make_shared<custom_interfaces::srv::FindClosestWall::Request>();
        auto future = find_closest_wall_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully moved to the closest wall.");
                // Create a subscription to /scan after robot arrives at the first wall
                scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "/scan", 10, std::bind(&WallFollowerNode::scan_callback, this, std::placeholders::_1));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to the closest wall.");
            }
        }
    }

    void WallFollowerNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges[0] < 0.5)
        {
            twist_.linear.x = 0.0;
            twist_.angular.z = -0.35;
        }
        else if (msg->ranges[25] < 0.55)
        {
            twist_.linear.x = 0.0;
            twist_.angular.z = -0.15;
        }
        else
        {
            twist_.linear.x = 0.5;
            twist_.angular.z = 0.0;
        }
        cmd_vel_publisher_->publish(twist_);
    }

    rclcpp::Client<custom_interfaces::srv::FindClosestWall>::SharedPtr find_closest_wall_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    geometry_msgs::msg::Twist twist_;
    float forward_distance_;
    float left_distance_;
    float right_distance_;
    float target_distance_;


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
