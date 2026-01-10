import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    init_robot_pose = launch_ros.actions.Node(
        package='wubot_application',
        executable='init_robot_pose',
    )
    get_robot_pose = launch_ros.actions.Node(
        package='wubot_application',
        executable='get_robot_pose',
    )
    nav_to_pose = launch_ros.actions.Node(
        package='wubot_application',
        executable='get_robot_pose',
    )
    return launch.LaunchDescription([
        init_robot_pose,
        # get_robot_node,
        nav_to_pose,
    ])
