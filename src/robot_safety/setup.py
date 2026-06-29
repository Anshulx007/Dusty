from setuptools import find_packages, setup
from glob import glob

package_name = "robot_safety"

setup(
    name=package_name,
    version="0.1.0",
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
    description="Safety and command arbitration node (stub)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_node = robot_safety.safety_node:main",
        ],
    },
)
