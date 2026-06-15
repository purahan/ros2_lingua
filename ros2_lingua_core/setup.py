from setuptools import find_packages, setup

setup(
    name="ros2_lingua_core",
    version="0.1.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "pytest-cov>=4.0",
            "pytest-timeout>=2.0",
        ]
    },
    zip_safe=True,
)
