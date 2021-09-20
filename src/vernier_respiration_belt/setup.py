from setuptools import setup
import os
from glob import glob


package_name = 'vernier_respiration_belt'

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
    description='it is to read the biosignals of the vernier respiration belt.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vernier_respiration_belt_node = vernier_respiration_belt.vernier_respiration_belt_node:main'
        ],
    },
)
