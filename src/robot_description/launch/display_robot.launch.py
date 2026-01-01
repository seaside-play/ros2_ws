import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('robot_description')
    default_model_path = urdf_tutorial_path + '/urdf/first_robot.urdf'
    # 为launch声明参数
    # 1. 定义可配置参数（对应 LaunchConfiguration('model')）
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=(default_model_path), description='URDF的绝对路径'
    )
    # 获取文件内容生成新的参数
    # 2. 你的核心代码：构建 robot_description 参数值
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    # 状态发布节点
    # 3. 发布 robot_description 参数（机器人状态发布节点，必须）
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]  # 将参数传入节点
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    # RViz节点
    default_rviz_config_path = urdf_tutorial_path + '/config/rviz/first_robot_urdf_display_model.rviz'
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments =['-d', default_rviz_config_path]
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])