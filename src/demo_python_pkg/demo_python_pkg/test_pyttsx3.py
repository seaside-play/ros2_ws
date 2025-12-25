#----------------------------------test pyttsx3 ----------------------------------
# import pyttsx3

# # 初始化引擎
# engine = pyttsx3.init()

# # 调整参数
# engine.setProperty('rate', 150)  # 语速（默认200，越小越慢）
# engine.setProperty('volume', 1.0)  # 音量（0.0-1.0）
# voices = engine.getProperty('voices')
# engine.setProperty('voice', voices[1].id)  # 切换语音（0=男声，1=女声，依系统而定）

# # 合成并播放
# engine.say("这是pyttsx3的合成语音，比eSpeak-NG自然很多")
# engine.say("支持离线运行，跨平台兼容性好")
# engine.runAndWait()

# 保存到文件（需安装pyobjc（macOS）/ pywin32（Windows））
# engine.save_to_file("保存语音到本地文件", "output.wav")
# engine.runAndWait()


#----------------------------------test TTS ----------------------------------
# from TTS.api import TTS

# # 初始化模型（选择中文预训练模型，首次运行自动下载）
# # 可选模型：tts_models/zh-CN/baker/tacotron2-DDC_ph（经典）、tts_models/zh-CN/jiaobei/vits（更自然）
# tts = TTS(model_name="tts_models/zh-CN/jiaobei/vits", progress_bar=False)

# # 合成语音到文件
# tts.tts_to_file(text="你好，这是Coqui TTS的中文合成语音，自然度远超eSpeak-NG", file_path="output.wav")

# # 直接播放（需安装pygame）
# # pip install pygame
# tts.tts_to_file(text="支持调整语速和音调", file_path="output2.wav", speed=1.2, pitch=1.1)


#!/usr/bin/env python3
"""ROS 2 语音合成节点（pyttsx3 实现，无依赖冲突）"""
import rclpy
from rclpy.node import Node
import pyttsx3
from std_msgs.msg import String  # 可选：订阅文本消息触发语音合成




class VoiceSynthesisNode(Node):
    def __init__(self):
        super().__init__("voice_synthesis_node")

        # 初始化 pyttsx3 引擎
        self.engine = pyttsx3.init()
        # 可选：配置语音参数（语速、音量、语音类型）
        self.engine.setProperty('rate', 150)  # 语速（默认200）
        self.engine.setProperty('volume', 1.0)  # 音量（0.0~1.0）
        # 可选：订阅文本话题，接收外部文本并合成语音

        voices = self.engine.getProperty('voices')
        # 切换为中文语音（需确保安装了中文语音包）
        for voice in voices:
            if "zh" in voice.languages or "Chinese" in voice.name:
                self.engine.setProperty('voice', voice.id)
                exit()

        self.text_sub = self.create_subscription(
            String,
            "voice_text_topic",  # 话题名
            self.text_callback,
            10  # 队列大小
        )
        # 初始测试语音
        self.speak("ROS 2 语音合成节点已启动，无任何依赖冲突！")
        self.get_logger().info("✅ 语音合成节点初始化完成")

    def text_callback(self, msg):
        """接收文本消息并合成语音"""
        text = msg.data
        self.get_logger().info(f"📢 合成语音：{text}")
        self.engine.say(text)
        self.engine.runAndWait()

    def speak(self, text):
        """直接合成语音（同步）"""
        self.engine.say(text)
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceSynthesisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点被手动终止")
    finally:
        # 释放资源
        node.engine.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
