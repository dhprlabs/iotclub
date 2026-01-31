from setuptools import find_packages, setup
from rosidl_cmake import build_type

package_name = 'practice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pub.launch.py']),
        ('share/' + package_name + '/srv', ['srv/AddTwoInts.srv']),  # Added service file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mangal-devanshu',
    maintainer_email='sarojmangal7600@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test = practice.test:main',
            'sub  = practice.subnode:main'
        ],
    },
)
