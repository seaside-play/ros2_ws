import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

# from launch.actions import ExecuteProcess, IfCondition, LogInfo, DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 功能四： 执行tf的静态发布和动态发布，之后再执行TF监听
    tf_broadcaster_static = launch_ros.actions.Node(
        package='demo_python_pkg',
        executable='tf_broadcaster_static',
        output='log',
    )
    tf_broadcaster_dynamic = launch_ros.actions.Node(
        package='demo_python_pkg',
        executable='tf_broadcaster_dynamic',
        output='log',
    )
    tf_listener = launch_ros.actions.Node(
        package='demo_python_pkg',
        executable='tf_listener',
        output='log',
    )

    launch_description = launch.LaunchDescription([tf_broadcaster_static, tf_broadcaster_dynamic, tf_listener])
    return launch_description

    # # 功能三： 结合替换来学习条件
    # # 1. 声明参数，是否创建新的海龟
    # declare_spawn_turtle = launch.actions.DeclareLaunchArgument(
    #     'spawn_turtle', default_value='false', description='是否生成海龟(true/false)'
    # )
    # spawn_turtle = launch.substitutions.LaunchConfiguration('spawn_turtle')
    # print(f'spawn_turtle {spawn_turtle} {IfCondition(spawn_turtle)}')
    # action_turtlesim = launch_ros.actions.Node(
    #     package='turtlesim', executable='turtlesim_node', name='turtlesim_node', output='screen'
    # )
    # # 2. 给日志输出和服务调用添加条件
    # action_executeprocess = launch.actions.ExecuteProcess(
    #     condition=IfCondition(spawn_turtle),
    #     cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '{x: 10, y: 10}']
    # )
    # action_log_info = launch.actions.LogInfo(
    #     condition=IfCondition(spawn_turtle),
    #     msg='使用executeprocess来调用服务生成海龟'
    # )
    # # 3 利用定时器动作实现依次启动
    # action_group = launch.actions.GroupAction([
    #     launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
    #     launch.actions.TimerAction(period=3.0, actions=[action_executeprocess]),
    # ])
    # # 4. 合成启动描述并返回
    # launch_description = launch.LaunchDescription([
    #     declare_spawn_turtle,
    #     action_turtlesim,
    #     action_executeprocess,
    #     action_log_info
    #     # action_group
    # ])

    # return launch_description
    # # 功能二: 包含其他launch文件，执行命令行，输出日志，使用GroupAction封装成组合
    # # 1. 利用IncludeLaunchDescription 动作包含其他launch文件
    # action_include_ launch = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(
    #         [get_package_share_directory("turtlesim"), "/launch", "/multisim.launch.py"]
    #     )
    # )
    # # 2. 利用ExecuteProcess 动作执行命令行
    # action_executeprocess = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'service', 'call', '/turtlesim/spawn', 'turtlesim/srv/Spawn', '{x: 1, y: 1}']
    # )
    # # 3. 利用LogInfo 动作输出日志
    # action_log_info = launch.actions.LogInfo(
    #     msg='使用launch来调用服务生成海龟')
    # # 4. 利用定时器动作实现依次启动日志输出和进程执行，并使用GroupAction封装组成
    # # 通过动作组统一管理这两个定时任务，简化 Launch 文件的逻辑组织。
    # action_group = launch.actions.GroupAction([
    #     launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
    #     launch.actions.TimerAction(period=3.0,  actions=[action_executeprocess]),
    # ])
    # # 5. 合成启动描述并返回
    # launch_description = launch.LaunchDescription([action_include_launch, action_group])
    # return launch_description

    # 功能一
    # # 创建参数声明action， 用于解析launch命令后的参数
    # # 动态赋值：支持两种赋值方式，灵活适配不同场景：
    # # 启动时通过命令行传入（ros2 launch 包名 launch文件.py 参数名:=参数值）；
    # # 在 Launch 文件内部通过 DeclareLaunchArgument 设定默认值；
    # #  “声明参数并设置默认值”；
    # # 关键搭配：DeclareLaunchArgument + LaunchConfiguration
    # action_declare_arg_max_speed = launch.actions.DeclareLaunchArgument('launch_max_speed', default_value='2.0')
    # action_node_turtle_control = launch_ros.actions.Node(
    #     package='demo_cpp_pkg',
    #     executable='turtle_control',
    #     # launch.substitutions.LaunchConfiguration:启动参数的灵活配置与传递，具体作用如下：
    #     # 1. 定义 / 引用 Launch 文件的可配置参数（支持启动时传入自定义值，无需硬编码）；
    #     # 2. 传递参数给节点、命令或其他 Launch 组件，实现 Launch 文件的复用性和灵活性
    #     # 引用参数
    #     parameters=[{'max_speed_': launch.substitutions.LaunchConfiguration('launch_max_speed', default='2.0')}],
    #     output='screen',
    # )
    # action_node_patrol_control = launch_ros.actions.Node(
    #     package='demo_cpp_pkg',
    #     executable='patrol_client',
    #     output='log',
    # )
    # action_node_turtlesim_node = launch_ros.actions.Node(
    #     package='turtlesim',
    #     executable='turtlesim_node',
    #     # output='both',
    #     output='screen',
    # )
    # # 合成启动描述并返回
    # launch_description = launch.LaunchDescription([
    #     action_declare_arg_max_speed,
    #     action_node_turtle_control,
    #     action_node_patrol_control,
    #     action_node_turtlesim_node
    # ])
    # return launch_description
