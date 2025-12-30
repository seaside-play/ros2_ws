#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node {
public:
    TFListener() : rclcpp::Node("tf_listener") {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        timer_ = create_wall_timer(5s, std::bind(&TFListener::get_transform, this));
    }

    void get_transform() {
        try {
            // 等待变换可用
            /*
            超时等待机制：rclcpp::Duration::from_seconds(1.0f) 是你提到的「等待变换可用」的关键配置，含义是：
            若调用函数时，缓冲区中已存在有效变换，直接返回结果，无需等待；
            若缓冲区中暂无有效变换，程序会阻塞等待，最长等待 1 秒；
            1 秒内仍未获取到有效变换（如帧不存在、变换链断裂），则抛出tf2_ros::TransformException异常，避免程序无限挂起。
            */
            const auto transform = buffer_->lookupTransform(
                "base_link", // 参数1：父坐标系（源坐标系，变换的起始帧）
                "target_point", // 参数2：子坐标系（目标坐标系，变换的终止帧）
                this->get_clock()->now(), // 参数3：查询的时间戳（此处为「当前最新时间」）
                rclcpp::Duration::from_seconds(1.0f) // 参数4：超时时间（等待1秒仍无变换则抛出异常）
            );
            // 转换结果并输出
            const auto& translation = transform.transform.translation;
            const auto& rotation = transform.transform.rotation;
            double yaw, pitch, roll;
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
            RCLCPP_INFO(get_logger(), "平移分量：(%f, %f, %f)", translation.x, translation.y, translation.z);
            RCLCPP_INFO(get_logger(), "旋转分量：(%f, %f, %f)", roll, pitch, yaw);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "异常: %s", ex.what());
        }
    }

private:
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;

}; // TFListener


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}