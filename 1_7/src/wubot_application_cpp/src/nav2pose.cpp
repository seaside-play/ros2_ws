#include <memory>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigationAction = nav2_msgs::action::NavigateToPose;

class NavToPoseClient : public rclcpp::Node {
public:
    // 定义导航动作客户端类型
    using NavigationActionClient = rclcpp_action::Client<NavigationAction>;
    // 定义导航动作目标句柄
    using NavigationActionGoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;

    NavToPoseClient() : Node("nav_to_pose_client") {
        action_client_ = rclcpp_action::create_client<NavigationAction>(this, "navigate_to_pose");
    }

    // 设置导航目标点和回调函数，即可向action服务端发送请求
    void sendGoal() {
        // 等待导航动作服务上线，等待时间为5s
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(get_logger(), "等待Action服务上线");
        }
        // 设置导航目标点
        auto goal_msg = NavigationAction::Goal();
        goal_msg.pose.header.frame_id = "map"; // 设置目标点的坐标系为地图坐标系
        goal_msg.pose.pose.position.x = 2.78;
        goal_msg.pose.pose.position.y = -0.87;

        auto send_goal_options = rclcpp_action::Client<NavigationAction>::SendGoalOptions();
        // 设置请求目标结果回调函数
        send_goal_options.goal_response_callback = [this](NavigationActionGoalHandle::SharedPtr goal_handle) -> void {
            if (goal_handle) {
                RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
            }
        };

        // 设置移动过程反馈回调函数
        send_goal_options.feedback_callback = [this](NavigationActionGoalHandle::SharedPtr goal_handle, 
                                                    const std::shared_ptr<const NavigationAction::Feedback> feedback) -> void {
            (void)goal_handle; // 假装调用，避免warning: unused
            RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f", feedback->distance_remaining);
        };

        // 设置执行结果回调函数
        send_goal_options.result_callback = [this](const NavigationActionGoalHandle::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "处理成功!");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);

    }

private:
    NavigationActionClient::SharedPtr action_client_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToPoseClient>();
    node->sendGoal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}