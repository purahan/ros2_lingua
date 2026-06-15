from setuptools import setup
import os
from glob import glob

package_name = "ros2_lingua_mock"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("lib", package_name, "dashboard"), glob("ros2_lingua_mock/dashboard/*")),
    ],
    install_requires=["setuptools"],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "balance_node      = ros2_lingua_mock.balance_node:main",
            "navigation_node   = ros2_lingua_mock.navigation_node:main",
            "manipulation_node = ros2_lingua_mock.manipulation_node:main",
            "speech_node       = ros2_lingua_mock.speech_node:main",
            "robot_monitor     = ros2_lingua_mock.robot_monitor:main",
            "cli               = ros2_lingua_mock.cli:main",
            "dashboard_server  = ros2_lingua_mock.dashboard_server:main",
        ],
    },
)
