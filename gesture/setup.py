from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'gesture_driven_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', 'gesture_driven_robot', 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='navya',
    maintainer_email='navvu.jain@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
    'console_scripts': [
        'vision_node = gesture_driven_robot.vision_node:main',
        'control_node = gesture_driven_robot.control_node:main',
    ],
},
)
