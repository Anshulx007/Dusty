from setuptools import find_packages, setup
import os
from glob import glob

package_name = "robot_driver"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anshul Tyagi",
    maintainer_email="anshul@robot.local",
    description="Arduino serial driver node",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arduino_driver_node = robot_driver.arduino_driver_node:main",
        ],
    },
)
