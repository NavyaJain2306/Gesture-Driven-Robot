import os
from setuptools import setup

package_name = 'gesture_driven_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['vision_node', 'control_node'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['robot_launch.py']),
        (os.path.join('share', package_name), ['robot.urdf', 'best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='navya',
    maintainer_email='navvu.jain@gmail.com',
    description='Gesture driven robot ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_node:main',
            'control_node = control_node:main',
        ],
    },
)
