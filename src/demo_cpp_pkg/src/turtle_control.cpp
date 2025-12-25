#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>

class TurtleControl : public rclcpp::Node {
public:
    TurtleControl() : Node("turtle_controller") {
        velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_subscription_ = create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, 
                                                             std::bind(&TurtleControl::on_pose_received, this, std::placeholders::_1));
    }

    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose) {
        // TODO: 收到位置计算误差，发布速度指令
        auto msg = geometry_msgs::msg::Twist();
        
        // 1. 记录当前位置
        double current_x = pose->x;
        double current_y = pose->y;
        RCLCPP_INFO(get_logger(), "当前位置: (x=%f, y=%f)", current_x, current_y);

        // 2. 计算与目标之间的距离，以及与当前海龟朝向的角度差
        double distance = std::sqrt((target_x_ - current_x)*(target_x_-current_x) + (target_y_-current_y)*(target_y_-current_y));
        // atan2全称为反正切函数（二参数版），核心作用是根据平面直角坐标系中某点的 y（纵坐标）和 x（横坐标）值，计算该点与原点连线和 x 轴正方向 的夹角（弧度制），
        double angle = std::atan2(target_y_-current_y, target_x_-current_x) - pose->theta; // 目标角度 - pose->theta 当前海龟的角度

        // 3. 控制策略：距离大于0.1继续运行，角度差大于0.2则原地旋转，否则直行；
        if (distance > 0.1) {
            // 从这个条件判断来看，每次发布速度控制，只调整其中一个变量，或线速度，或角度
            if (std::fabs(angle) > 0.2) {
                msg.angular.z = fabs(angle); 
            } else {
                // 通过比例控制器计算输出速度
                msg.linear.x = k_ * distance;
            }
        }

        // 4. 限制最大值并发布消息
        if (msg.linear.x > max_speed_) {
            msg.linear.x = max_speed_;
        } 
        velocity_publisher_->publish(msg);
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    double target_x_{10.0};
    double target_y_{8.0};
    double k_{1.0}; // 比例系数，控制输出=误差*比例系数
    double max_speed_{3.0}; // 最大线速度，设置默认值为3.0
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}