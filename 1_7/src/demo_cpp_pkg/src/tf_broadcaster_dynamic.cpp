#include <chrono>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
 // 1. 包含必要头文件
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TFBroadcasterDynamic : public rclcpp::Node {
public:
    TFBroadcasterDynamic() : rclcpp::Node("tf_broadcaster_dynamic_node") {
        // 2. 创建动态变换广播器（关键：用于发布变换数据）
        transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(10ms, std::bind(&TFBroadcasterDynamic::publish_transform, this));
    }

    void publish_transform() {
        // 3. 构建变换数据（你只做了这一步的帧名定义，缺少后续发布步骤）
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map_1";
        transform.child_frame_id = "base_link";

         // 补充：必须定义具体的平移/旋转参数（否则变换无效）
        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 30 * M_PI / 180);
        transform.transform.rotation = tf2::toMsg(quat);

        // 4. 发布变换数据（核心缺失步骤！这一步才会让帧被系统感知）
        transform_broadcaster_->sendTransform(transform);
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFBroadcasterDynamic>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}