// Helper node that periodically clears Nav2 costmaps around a robot.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"

class ClearCostmapClient : public rclcpp::Node
{
public:
    using ClearCostmap = nav2_msgs::srv::ClearCostmapAroundRobot;

    ClearCostmapClient()
    : Node("clear_costmap")
    {
        client_global_ = this->create_client<ClearCostmap>("global_costmap/clear_around_global_costmap");
        RCLCPP_INFO(this->get_logger(), "Waiting for service: %s", client_global_->get_service_name());
        timer_global_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ClearCostmapClient::send_request_global_, this)
        );

        client_local_ = this->create_client<ClearCostmap>("local_costmap/clear_around_local_costmap");
        RCLCPP_INFO(this->get_logger(), "Waiting for service: %s", client_local_->get_service_name());
        timer_local_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ClearCostmapClient::send_request_local_, this)
        );
    }
private:
    rclcpp::Client<ClearCostmap>::SharedPtr client_global_;
    rclcpp::Client<ClearCostmap>::SharedPtr client_local_;
    rclcpp::TimerBase::SharedPtr timer_global_;
    rclcpp::TimerBase::SharedPtr timer_local_;

    void send_request_global_()
    {
        if (!client_global_->wait_for_service(std::chrono::seconds(1))) {
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            return;
        }

        auto request = std::make_shared<ClearCostmap::Request>();
        request->reset_distance = 3.0;

        auto result_future = client_global_->async_send_request(request,
            [this](rclcpp::Client<ClearCostmap>::SharedFuture future) {
                try {
                    auto response = future.get();
                    // RCLCPP_INFO(this->get_logger(), "Costmap cleared");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }
    void send_request_local_()
    {
        if (!client_local_->wait_for_service(std::chrono::seconds(1))) {
            // RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            return;
        }

        auto request = std::make_shared<ClearCostmap::Request>();
        request->reset_distance = 3.0;

        auto result_future = client_local_->async_send_request(request,
            [this](rclcpp::Client<ClearCostmap>::SharedFuture future) {
                try {
                    auto response = future.get();
                    // RCLCPP_INFO(this->get_logger(), "Costmap cleared");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClearCostmapClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
