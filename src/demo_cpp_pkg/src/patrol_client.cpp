#include <chrono>
#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "image_interfaces/srv/patrol.hpp"

using namespace std::chrono_literals;
using Patrol = image_interfaces::srv::Patrol;

class PatrolClient : public rclcpp::Node {
public:
    PatrolClient() : Node("patrol_client") {
        patrol_client_ = create_client<Patrol>("patrol");
        // 壁钟定时器的核心 API（主要用于 C++/Python 节点），作用是按指定时间间隔重复执行自定义回调函数
        // 是实现 “定时任务” 的关键工具。
        timer_ = create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
        srand(time(NULL));
    }

    // 生成随机目标点，请求服务端
    void timer_callback() {
        // 1. 等待服务端上线
        while (!patrol_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被中断...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务端上线...");
        }

        // 2. 构造请求的对象
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 10;
        request->target_y = rand() % 10;
        RCLCPP_INFO(get_logger(), "请求巡逻：(%f, %f)", request->target_x, request->target_y);

        // 3. 发送异步请求，然后等待返回，返回时调用回调函数
        patrol_client_->async_send_request(request, 
                                        [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void {
                                            auto response = result_future.get();
                                            if (response->result == Patrol::Response::SUCCESS) {
                                                RCLCPP_INFO(get_logger(), "目标点处理成功");
                                            } else if (response->result == Patrol::Response::FAIL) {
                                                RCLCPP_INFO(get_logger(), "目标点处理失败");
                                            } 
                                           });

    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




