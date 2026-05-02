from setuptools import setup, find_packages

setup(
    name="ros2_lingua_core",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/ros2_lingua_core"]),
        ("share/ros2_lingua_core", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
