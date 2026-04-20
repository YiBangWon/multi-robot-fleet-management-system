// Bridge from FMS route messages to Nav2 NavigateThroughPoses goals.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
int resolveNamespaceIndex(const std::string &robot_namespace)
{
    if (robot_namespace == "/tb1") return 0;
    if (robot_namespace == "/tb2") return 1;
    if (robot_namespace == "/tb3") return 2;
    if (robot_namespace == "/tb4") return 3;
    if (robot_namespace == "/tb5") return 4;
    if (robot_namespace == "/tb6") return 5;
    return 0;
}
}  // namespace

class SendingGoalsClient : public rclcpp::Node
{
public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleSendingGoals = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    SendingGoalsClient()
        : Node("nav2_ctrl_route"),
          namespace_index_(resolveNamespaceIndex(this->get_namespace()))
    {
        // Forward the latest route message to Nav2's NavigateThroughPoses action.
        client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
            this, "navigate_through_poses");

        subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "route",
            10,
            std::bind(&SendingGoalsClient::topic_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;
    int namespace_index_{0};

    void topic_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        poses_ = msg->poses;
        RCLCPP_INFO(
            this->get_logger(),
            "Robot namespace index %d received %zu waypoints.",
            namespace_index_,
            poses_.size());

        send_goals();
    }

    void send_goals()
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses action server is unavailable");
            return;
        }

        auto goal_msg = NavigateThroughPoses::Goal();
        goal_msg.poses = poses_;

        RCLCPP_INFO(this->get_logger(), "Sending goal to robot");

        auto send_goal_options =
            rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(
            &SendingGoalsClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(
            &SendingGoalsClient::feedback_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2);
        send_goal_options.result_callback = std::bind(
            &SendingGoalsClient::result_callback, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(GoalHandleSendingGoals::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleSendingGoals::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Remain distance: %.2f", feedback->distance_remaining);
    }

    void result_callback(const GoalHandleSendingGoals::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendingGoalsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
