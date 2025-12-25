cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

# 关键：手动指定 MRPT 的 CMake 配置目录（必须在 find_package(MRPT) 之前）
# set(MRPT_DIR "/usr/local/lib/cmake/" CACHE PATH "MRPT CMake 配置目录")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(status_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(cpp_node src/person_node.cpp)
add_executable(turtle_circle src/turtle_circle.cpp)
add_executable(turtle_control src/turtle_control.cpp)
add_executable(hello_qt src/hello_qt.cpp)
add_executable(sys_status_display src/sys_status_display.cpp)

target_link_libraries(hello_qt Qt5::Widgets)
target_link_libraries(sys_status_display Qt5::Widgets)

ament_target_dependencies(cpp_node rclcpp)
ament_target_dependencies(turtle_circle rclcpp geometry_msgs turtlesim )
ament_target_dependencies(turtle_control rclcpp geometry_msgs turtlesim )
ament_target_dependencies(sys_status_display rclcpp status_interfaces)

install(TARGETS
cpp_node
turtle_circle
turtle_control
hello_qt
sys_status_display
DESTINATION lib/${PROJECT_NAME}
)

# find_package(MRPT 2.0 REQUIRED COMPONENTS
#   core       # 核心功能
#   slam       # SLAM 算法
#   maps       # 地图模块
#   obs        # 传感器数据处理
#   gui        # 可视化（可选）
#   opengl     # 3D 可视化（可选）
# )

# if(MRPT_FOUND)
#   message(STATUS "MRPT 包找到")
# else()
#   message(WARNING "MRPT 沒有找到")
# endif()

# target_link_libraries(cpp_node
#   MRPT::core
#   MRPT::slam
#   MRPT::maps
#   MRPT::obs
#   MRPT::gui  # 若启用了 gui 组件
# )

# 链接依赖（ROS 2 + MRPT2）

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 从CMakeLists.txt收集信息
ament_package()
