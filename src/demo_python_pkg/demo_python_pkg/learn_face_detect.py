import face_recognition # from就像是include "1.hpp"一样，编译后会加载到当前原文件中，
import cv2              # 而import就是将对应的类或函数，导入到当前py源文件中，
                        # 以便在源文件中可以直接调用import的类或函数
from ament_index_python.packages import get_package_share_directory
import os

def main():
    # 1. 获取图片真实路径
    default_image_path = get_package_share_directory('demo_python_pkg') + '/resource/default.jpeg'

    # 1.1 检查文件是否存在
    if not os.path.exists(default_image_path):
        print(f"错误：图片文件不存在 → {default_image_path}")
        return 1

    # 2. 使用opencv加载图像
    image = cv2.imread(default_image_path)

    # 3. 查找图像中的所有人脸
    # number_of_times_to_upsample=1 采样次数，次数越多越能发现图像中更小的人脸。
    # model='hog' hog模型：优势速度快，但准确性不高， cnn模型：更准确，但用CPU检测速度会慢
    face_locations = face_recognition.face_locations(image, number_of_times_to_upsample=1, model='hog')

    print(f"检测到 {len(face_locations)} 个人脸")

    # 4. 绘制每个人脸的边框
    for top, right, bottom, left in face_locations: # 列表，内容是元组
        cv2.rectangle(image, (left, top), (right, bottom), (0,0,255), 4) # (255,0,0)：用元组表示颜色（蓝，绿，红），4：表示线的粗细

    # 5. 显示结果图像
    cv2.imshow('Face Detection', image)
    cv2.waitKey(0)