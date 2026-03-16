import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration
from autopatrol_interfaces.srv import SpeechText
# 导入消息接口和相关接口
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

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

        # 订阅与保存图像相关定义
        self.declare_parameter('image_save_path', '')
        self.image_save_path_ = self.get_parameter('image_save_path').value
        self.bridge_ = CvBridge()
        self.latest_image_= None
        # 订阅摄像头图像话题，获取最新图像数据，并在回调函数中保存到latest_image_变量中，以便后续使用。
        # 只要订阅了，就会一直回调处理。
        self.subscription_image_ = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)

    # 将最新的消息放到latest_image中
    def image_callback(self, msg):
        self.latest_image_ = msg

    # 通过x, y, yaw合成PostStamped
    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw) # roll, pitch, yaw
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
        self.waitUntilNav2Active() # 等待 Nav2 导航系统完全启动、准备就绪，直到可以接收目标点 
                                   # 大白话：“等导航准备好，我再往下执行！
 
    # 准备获取所有的目标点数据，通过参数值获取目标点集合，之后再逐一对数据进行解析和使用。
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

    # 导航到指定位姿，执行一次导航，并获得结果
    def nav_to_pose(self, target_pose):
        self.waitUntilNav2Active() # 在正式导航前，需要等待nav2激活。
        result = self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                # Duration.from_msg： 把 ROS 消息格式的时间数据，转换成程序能用的 Duration 时长对象。
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

    # 记录图像
    def record_image(self):
        if self.latest_image_ is not None:
            pose = self.get_current_pose() 
            # imgmsg_to_cv2 把 ROS 图像消息（sensor_msgs/Image）转换成 OpenCV 能处理的图像格式（cv::Mat）
            cv_image = self.bridge_.imgmsg_to_cv2(self.latest_image_)
            # x:3.2f 表示总长度3，小数点后2位
            cv2.imwrite(f'{self.image_save_path_}image_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png', cv_image)

    # 通过TF获取当前位姿
    def get_current_pose(self):
        while rclpy.ok():
            try:
                # 获取从地图 map 坐标系 到 机器人底盘 base_footprint 坐标系的坐标变换（也就是机器人在地图里的实时位置 + 朝向）。
                tf = self.buffer_.lookup_transform('map',  # 目标坐标系（我要转到哪里）， 我想得到 相对于地图 的位置
                                                   'base_footprint',  # # 源坐标系（从哪里开始转）
                                                   rclpy.time.Time(seconds=0), # 查询哪个时间的坐标变换
                                                   rclpy.time.Duration(seconds=1)) # 最多等多久
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
            
            # 记录图像
            patrol_node.speech_text(f'已到目标点{x}, {y}, 准备记录图像')
            patrol_node.record_image()
            patrol_node.speech_text('图像保存完成')
    rclpy.shutdown()