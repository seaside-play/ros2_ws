import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    
    # 创建点集
    goal_poses = []
    
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map' # 这是什么意思呢？
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.64
    goal_pose1.pose.position.y = 1.14
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 3.48
    goal_pose2.pose.position.y = 1.35
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 2.78
    goal_pose3.pose.position.y = -0.87
    goal_pose3.pose.orientation.w = 1.0
    goal_poses.append(goal_pose3)

    # 调用路点导航服务
    navigator.followWaypoints(goal_poses)

    # 判断结束及获取反馈
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'当前目标编号：{feedback.current_waypoint}')

    # 最终结果判断
    result = navigator.getResult()
    if result == result.SUCCEEDED:
        navigator.get_logger().info('导航结果： 成功')
    elif result == result.CANCELED:
        navigator.get_logger().warn('导航结果： 取消')
    elif result == result.FAILED:
        navigator.get_logger().error('导航结果：失败')
    else:
        navigator.get_logger().error('导航结果:返回状态无效')
    