import os
from glob import glob
from setuptools import setup

package_name = 'ros2_lingua'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ros2_lingua']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'grounding_node = ros2_lingua.grounding_node:main',
            'dispatcher_node = ros2_lingua.dispatcher_node:main',
        ],
    },
)
