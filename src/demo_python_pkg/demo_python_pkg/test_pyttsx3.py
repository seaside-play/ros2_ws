import pyttsx3

# 初始化引擎
engine = pyttsx3.init()

# 调整参数
engine.setProperty('rate', 150)  # 语速（默认200，越小越慢）
engine.setProperty('volume', 1.0)  # 音量（0.0-1.0）
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[1].id)  # 切换语音（0=男声，1=女声，依系统而定）

# 合成并播放
engine.say("这是pyttsx3的合成语音，比eSpeak-NG自然很多")
engine.say("支持离线运行，跨平台兼容性好")
engine.runAndWait()

# 保存到文件（需安装pyobjc（macOS）/ pywin32（Windows））
# engine.save_to_file("保存语音到本地文件", "output.wav")
# engine.runAndWait()
