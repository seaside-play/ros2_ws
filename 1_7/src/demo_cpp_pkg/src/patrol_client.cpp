#include <chrono>
#include <cstdlib>
#include <ctime>
#include <rcl_interfaces/msg/detail/parameter_value__struct.hpp>
#include <rcl_interfaces/srv/detail/set_parameters__struct.hpp>
#include <rclcpp/utilities.hpp>
#include "rclcpp/rclcpp.hpp"
#include "image_interfaces/srv/patrol.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using namespace std::chrono_literals;
using Patrol = image_interfaces::srv::Patrol;
using SetParameters = rcl_interfaces::srv::SetParameters;

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

    // 20251229. 001:修改其它节点参数
    std::shared_ptr<SetParameters::Response> call_set_parameters(rcl_interfaces::msg::Parameter &parameter) {
        // 1. 创建客户端等待服务上线
        auto param_client = this->create_client<SetParameters>("/turtle_controller/set_parameters");
        while (!param_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被中断...");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "等待参数服务端上线...");
        }
        // 2. 创建请求对象
        auto request = std::make_shared<SetParameters::Request>();
        request->parameters.push_back(parameter);
        // 3. 异步调用、等待并返回响应结果
        auto future = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto result = future.get();
        return result;
    }

    void udpate_server_param_k(double k) {
        // 1. 创建一个参数对象
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k_";
        // 2. 创建参数值对象并赋值
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;
        // 3. 请求更新参数并处理
        auto response = call_set_parameters(param);
        if (response == nullptr) {
            RCLCPP_WARN(this->get_logger(), "参数修改失败");
            return;
        } else {
            for (const auto& result : response->results) {
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), "参数k 已修改为: %f", k);
                } else {
                    RCLCPP_WARN(this->get_logger(), "参数k  失败原因：%s", result.reason.c_str());
                }
            }
        }
    }
    // 20251229. End

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    // 20251229. 002:修改其它节点参数
    node->udpate_server_param_k(1.5);
    // 20251229. End
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




