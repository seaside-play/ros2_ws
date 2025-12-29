import launch
import launch_ros

def generate_launch_description():
    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_pkg',
        executable='turtle_control',
        output='screen',
    )
    action_node_patrol_control = launch_ros.actions.Node(
        package='demo_cpp_pkg',
        executable='patrol_client',
        output='log',
    )
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        # output='both',
        output='screen',
    )
    # 合成启动描述并返回
    launch_description = launch.LaunchDescription([
        action_node_turtle_control,
        action_node_patrol_control,
        action_node_turtlesim_node
    ])
    return launch_description
