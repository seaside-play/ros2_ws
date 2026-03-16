import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import threading
from queue import Queue
import time
# import espeakng
import pyttsx3

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.speak = pyttsx3.init()
        self.speak.setProperty('rate', 150) # 语速（默认200，越小越慢）
        self.speak.setProperty('volumn', 1.0) # 音量（0.0~1.0）
        self.voices = self.speak.getProperty('voices')
        self.speak.setProperty('voice', self.voices[1].id) # 切换语音（0-男生，1-女生，依系统而定）
        
        self.novel_queue_ = Queue()
        self.novel_subscriber_ = self.create_subscription(String, 'novel', self.novel_callback, 10)
        self.speech_thread_ = threading.Thread(target=self.speak_thread)
        self.speech_thread_.start()

    def novel_callback(self, msg):
        self.novel_queue_.put(msg.data)

    # 使用pyttsx3引擎发音
    def speak_thread(self):
        while rclpy.ok():
            if self.novel_queue_.qsize() > 0:
                text = self.novel_queue_.get()
                self.get_logger().info(f'正在朗读{text}')
                self.speak.say(text)
                # self.speak.runAndWait()
            else:
                time.sleep(1)

    # 使用espeakng引擎发音
    # def speak_thread(self):
    #     speaker = espeakng.Speaker()
    #     speaker.voice = 'zh' # 说明可以选择各种语言
    #     while rclpy.ok():
    #         if self.novel_queue_.qsize() > 0:
    #             text = self.novel_queue_.get()
    #             self.get_logger().info(f'正在朗读{text}')
    #             speaker.say(text)
    #             speaker.wait()
    #         else:
    #             time.sleep(1)

def main():
    rclpy.init()
    node = NovelSubNode('novel_read')
    rclpy.spin(node)
    rclpy.shutdown()
