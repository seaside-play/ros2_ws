import rclpy
from rclpy.node import Node
from image_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image # 图像对象在sensor_msgs.msg下
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
# 功能二
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class FaceDetectionClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.client_ = self.create_client(FaceDetector, '/face_detect') # 指定服务的名字以及服务接口类型
        self.bridge_ = CvBridge()
        self.test1_image_path_ = get_package_share_directory('demo_python_pkg') + '/resource/test1.jpeg'
        self.image_ = cv2.imread(self.test1_image_path_)
    
    # 发送请求并处理结果
    def send_request(self):
        # 功能一：发送请求，获取图像中人脸的坐标位置
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
        # self.show_face_locations(response)

        # 功能二
        pass

    def show_face_locations(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image_, (left, top), (right, bottom), (0,0,255), 2)
        cv2.imshow('Face Detection Result', self.image_)
        cv2.waitKey(0)

    # 功能二
    def call_set_parameters(self, parameters):
        # 1. 创建一个客户端，并等待服务上线
        client = self.create_client(SetParameters, '/face_detection_node/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待参数服务器上线...')
        
        # 2. 创建请求对象
        request = SetParameters.Request()
        # SetParamter的参数是数组，所以调用者使用数组的信息，可通过ros2 interface show 查看请求和响应的结构体内容
        request.parameters = parameters 
        
        # 3. 异步调用、等待并返回响应结果
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response
    
    def update_detect_model(self, model):
        # 1. 创建一个参数对象
        param = Parameter()
        param.name = 'face_locations_model'
        # 2. 创建参数值对象并赋值
        new_model_value = ParameterValue()
        new_model_value.type = ParameterType.PARAMETER_STRING
        new_model_value.string_value = model
        param.value = new_model_value
        # 3. 请求更新参数并处理
        response = self.call_set_parameters([param]) # 参数以列表的形式存放
        for result in response.results:
            if result.successful:
                self.get_logger().info(f'参数{param.name} 设置为{model}')
            else:
                self.get_logger().info(f'参数设置失败，原因为：{result.reason}')

def main(args=None):
    rclpy.init(args=args)
    # 功能一
    # node = FaceDetectionClient()
    # node.send_request()

    # 功能二
    face_detect_client = FaceDetectionClient()
    face_detect_client.update_detect_model('hog')
    face_detect_client.send_request()
    face_detect_client.update_detect_model('cnn')
    face_detect_client.send_request()
    rclpy.spin(face_detect_client)
    rclpy.shutdown()