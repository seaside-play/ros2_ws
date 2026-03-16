import rclpy
from rclpy.node import Node

def main():
  rclpy.init() # 分配通信资源
  node = Node("python_node") # 创建节点，用于发布或订阅话题等
  node.get_logger().info('Hello python_node') # 获取logger对象，并答应info信息
  rclpy.spin(node) # 启动节点
  rclpy.shutdown() # 清理分配的资源