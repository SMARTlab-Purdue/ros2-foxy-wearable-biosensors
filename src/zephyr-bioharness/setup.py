from setuptools import setup
import os
from glob import glob


package_name = 'zephyr-bioharness'

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
    maintainer='Robert Wilson',
    maintainer_email='robert.wilson@ucsc.edu',
    description='it is to read the biosignals of the zephyr bioharness.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zephyr-bioharness_node = zephyr-bioharness.zephyr-bioharness_node:main'
        ],
    },
)
