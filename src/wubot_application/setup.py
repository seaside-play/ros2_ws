import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'wubot_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='namtaerg@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'init_robot_pose = wubot_application.init_robot_pose:main',
            'get_robot_pose = wubot_application.get_robot_pose:main',
            'nav_to_pose = wubot_application.nav_to_pose:main',
        ],
    },
)
