from setuptools import setup
import os
from glob import glob

package_name = 'empatica_e4'

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
    maintainer='Wonse Jo',
    maintainer_email='wonsu0513@gmail.com',
    description='it is to read the biosignals of the empatica e4.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'empatica_e4_node = empatica_e4.empatica_e4_node:main'
        ],
    },
)
