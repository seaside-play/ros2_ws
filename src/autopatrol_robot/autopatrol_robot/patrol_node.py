import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration
from autopatrol_interfaces.srv import SpeechText

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 2.86, -0.87, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value

        # 实时位置获取TF相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        # 语音合成客户端
        self.speech_client_ = self.create_client(SpeechText, 'speech_text')

    # 通过x, y, yaw合成PostStamped
    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose

    # 初始化机器人位姿
    def init_robot_pose(self):
        self.initial_point_ = self.get_parameter('initial_point').value
        self.setInitialPose(self.get_pose_by_xyyaw(self.initial_point_[0],
                                                   self.initial_point_[1],
                                                   self.initial_point_[2]))
        self.waitUntilNav2Active()
 
    # 通过参数值获取目标点集合
    def get_target_points(self):
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取目标点：{index} -> ({x}, {y}, {yaw})')
        return points

    # 导航到指定位姿
    def nav_to_pose(self, target_pose):
        self.waitUntilNav2Active()
        result = self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f'预计:{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 到达')
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果:成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果:取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果:失败')
        else:
            self.get_logger().error('导航结果:返回状态无效')

    # 通过TF获取当前位姿
    def get_current_pose(self):
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform('map', 
                                                   'base_footprint', 
                                                   rclpy.time.Time(seconds=0),
                                                   rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(f'平移:{transform.translation}, 旋转四元数:{transform.rotation}, 旋转欧拉角:{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因{str(e)}')
        
    # 调用服务播放语音
    def speech_text(self, text):
        while not self.speech_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音播放服务未上线，等待中...')

        request = SpeechText.Request()
        request.text = text
        future = self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result().result
            if result:
                self.get_logger().info(f'语音合成为：{text}')
            else:
                self.get_logger().warn(f'语音合成失败：{text}')
        else:
            self.get_logger().warn('语音合成服务请求失败')


def main():
    rclpy.init()
    patrol_node = PatrolNode()
    patrol_node.speech_text('正在初始化位置')
    patrol_node.init_robot_pose()
    patrol_node.speech_text('位置初始化完成')

    while rclpy.ok():
        points = patrol_node.get_target_points()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol_node.get_pose_by_xyyaw(x, y, yaw)
            patrol_node.speech_text(f'准备前往目标点{x}, {y}')
            patrol_node.nav_to_pose(target_pose)
    rclpy.shutdown()