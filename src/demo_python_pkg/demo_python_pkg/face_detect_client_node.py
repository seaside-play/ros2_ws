import rclpy
from rclpy.node import Node
from image_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image # 图像对象在sensor_msgs.msg下
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge

class FaceDetectionClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.client_ = self.create_client(FaceDetector, '/face_detect') # 指定服务的名字以及服务接口类型
        self.bridge_ = CvBridge()
        self.test1_image_path_ = get_package_share_directory('demo_python_pkg') + '/resource/test1.jpeg'
        self.image_ = cv2.imread(self.test1_image_path_)
    
    # 发送请求并处理结果
    def send_request(self):
        # 1. 判断服务是否在线
        # 阻塞式（或定时等待式）检查指定名称的服务端是否已启动并注册到 ROS 2 系统中（即服务是否可用）
        while self.client_.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'等待服务端上线...')

        # 2. 构建request
        request = FaceDetector.Request()
        request.image = self.bridge_.cv2_to_imgmsg(self.image_)

        # 3. 发送并spin等待服务处理完成
        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # 4. 展现处理结果
        response = future.result()
        self.get_logger().info(f'接收到响应： 图像中共有:{response.number}张脸， 耗时{response.use_time}')
        self.show_face_locations(response)

    def show_face_locations(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image_, (left, top), (right, bottom), (0,0,255), 2)
        cv2.imshow('Face Detection Result', self.image_)
        cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionClient()
    node.send_request()
    rclpy.shutdown()