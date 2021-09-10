import os
from glob import glob
from setuptools import setup

package_name = 'ros2-foxy-wearable-biosensors'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='purdue-ucsc',
    maintainer_email='wonsu0513@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ROS2_Emotiv_Insight_Node = ros2-foxy-wearable-biosensors.emotiv_insight.emotiv_insight_node:main',
            'ROS2_Empatica_E4_Node = ros2-foxy-wearable-biosensors.empatica_e4.empatica_e4_node:main',
            'ROS2_Vernier_Respiration_Belt_Node = ros2-foxy-wearable-biosensors.vernier_respiration_belt.vernier_respiration_belt_node:main',
            'ROS2_Polar_H10_Node = ros2-foxy-wearable-biosensors.polar_h10.polar_h10_node:main',
        ],
    },
)
