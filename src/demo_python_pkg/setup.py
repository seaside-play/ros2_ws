from setuptools import find_packages, setup

package_name = 'demo_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'person_node1 = demo_python_pkg.person_node:main',
            'python_node = demo_python_pkg.python_node:main',
            'novel_pub = demo_python_pkg.novel_pub_node:main',
            'novel_read = demo_python_pkg.novel_sub_node:main',
            'sys_status_pub = demo_python_pkg.sys_status_pub:main',
        ],
    },
)
