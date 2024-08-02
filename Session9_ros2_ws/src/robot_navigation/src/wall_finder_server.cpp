#include "wall_finder_server.h"



    WallFinderServer::WallFinderServer() : Node("wall_finder_server")
    {
        service_ = this->create_service<custom_interfaces::srv::FindClosestWall>(
            "wall_finder", std::bind(&WallFinderServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFinderServer::laser_scan_callback, this, std::placeholders::_1));

        service_called=false;
    }


    void WallFinderServer::handle_service(
        const std::shared_ptr<custom_interfaces::srv::FindClosestWall::Request> request,
        std::shared_ptr<custom_interfaces::srv::FindClosestWall::Response> response)
    {
        service_called=true;
        response->success = true;             
    }

    void WallFinderServer::laser_scan_callback(const sensor_msgs::msg::LaserScan msg)
    {
     float min_distance = msg.ranges[0];
     distance = min_distance;
     RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Distance recieved");
     auto command = geometry_msgs::msg::Twist();
    if (service_called){
        if(distance > 0.5){
            command.linear.x = 0.2; 
            cmd_vel_publisher_->publish(command);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Robot moving to wall");  
              
        } 
        else{
            command.linear.x = 0.0;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Wall detected");  
            cmd_vel_publisher_->publish(command);
            rclcpp::shutdown();
            // try putting service_called=false;

        }

    }
    return;
    }

    rclcpp::Service<custom_interfaces::srv::FindClosestWall>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFinderServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
