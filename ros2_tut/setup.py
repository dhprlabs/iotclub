from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_tut'
serv_pkg_name = 'ros2_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', serv_pkg_name, 'srv'), glob('srv/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pawan',
    maintainer_email='pawankumar27112005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = ros2_tut.my_node:main',
            'rclpy_node = ros2_tut.rclpy_node:main',
            'publisher_node = ros2_tut.publisher_node:main',
            'publisher_oop = ros2_tut.publisher_oop:main',
            'subscriber_node = ros2_tut.subscriber_node:main',
            'subscriber_oop = ros2_tut.subscriber_oop:main',
            'service_server = ros2_tut.service_server:main',
            'service_client = ros2_tut.service_client:main',
        ],
    },
)
