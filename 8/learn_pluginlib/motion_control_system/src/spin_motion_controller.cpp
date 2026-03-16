#include <iostream>
#include "motion_control_system/spin_motion_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace motion_control_system {

void SpinMotionController::start() {
    // 实现旋转运动控制逻辑
    std::cout << "SpinMotionController::start" << std::endl;
}

void SpinMotionController::stop() {
    // 停止运动控制
    std::cout << "SpinMotionController::stop" << std::endl;
}

} // namespace motion_control_system

// 导出插件，使其可以被ROS插件库加载，并且指定它实现了MotionController接口。
// 第一个是要导出的类，第二个事导出的类的抽象基类
// 自此，编写好了插件的代码，接下来要编写插件的XML描述文件，告诉ROS插件库如何加载这个插件，以及它实现了哪个接口。
PLUGINLIB_EXPORT_CLASS(motion_control_system::SpinMotionController, motion_control_system::MotionController)