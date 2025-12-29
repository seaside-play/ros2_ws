from setuptools import find_packages, setup
from glob import glob

package_name = 'demo_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 目标路径，源文件路径， 编译时为创建目标路径，并将源文件路径上的文件复制到目标路径中
        # resource/default.jpeg仅仅是指明源文件的路径，而拷贝的只是defautl.jpeg文件到目标文件夹下
        ('share/' + package_name + "/resource", ['resource/default.jpeg', 'resource/test1.jpeg']), 
        ('share/' + package_name + "/launch", glob('launch/*.launch.py')), # 通过正则匹配所有文件
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='namtaerg@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 实现可执行程序名称(可以是随意取名哦)和main函数的关联（关联是重点），尤其是learn_face_detect
            'person_node1 = demo_python_pkg.person_node:main',
            'python_node = demo_python_pkg.python_node:main',
            'novel_pub = demo_python_pkg.novel_pub_node:main',
            'novel_read = demo_python_pkg.novel_sub_node:main',
            'sys_status_pub = demo_python_pkg.sys_status_pub:main',
            'learn_face_detect = demo_python_pkg.learn_face_detect:main',
            'face_detect_node = demo_python_pkg.face_detect_node:main',
            'face_detect_client_node = demo_python_pkg.face_detect_client_node:main',
        ],
    },
)
