# 人脸检测服务的实现
import rclpy
from rclpy.node import Node
from image_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # 用于转化格式
import cv2
import face_recognition
import time
import os

class FaceDectectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.bridge_ = CvBridge()
        # 使用create_service创建一个服务，有三个参数，依次分别是：消息接口类型，服务名称
        self.service_ = self.create_service(FaceDetector, '/face_detect', self.detect_face_callback)
        self.default_image_path_ = get_package_share_directory('demo_python_pkg')+'/resource/default.jpeg'
        
        # 检查文件是否存在
        if not os.path.exists(self.default_image_path_):
            print(f"错误：图片文件不存在 → {self.default_image_path_}")
            return
        
        self.upsample_times_ = 1 # 默认采样次数
        self.model_ = 'cnn' #'hog' # 默认模型

    # 完成人脸检测
    def detect_face_callback(self, request, response):
        if request.image.data: # 判断数组是否有值的方法
            cv_image = self.bridge_.imgmsg_to_cv2(request.image) # 将ros 2 的image转换位opence能够识别的image
        else:
            cv_image = cv2.imread(self.default_image_path_) # 若客户端请求的image为空，则直接读取默认图像
        start_time = time.time()
        self.get_logger().info('加载图像，开始检测')
        face_locations = face_recognition.face_locations(cv_image, 
                                                         number_of_times_to_upsample=self.upsample_times_,
                                                         model=self.model_)
        end_time = time.time()
        self.get_logger().info(f'检测完成，耗时{end_time - start_time}')
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top) # 为何int32[]的数组形式，可以使用append呢？因为在python中对应了list，其函数就是append
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = FaceDectectionNode()
    rclpy.spin(node) # 事件处理
    rclpy.shutdown()
