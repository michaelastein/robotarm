from setuptools import find_packages, setup
from glob import glob
import os

package_name = "robotarm_software"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*"),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="admin",
    maintainer_email="admin@example.com",
    description="Custom robot arm software without MoveIt Servo",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_node.py = robotarm_software.safety_node:main",
            "safety_supervisor.py = robotarm_software.safety_supervisor:main",
            "tool_position_velocity_control.py = robotarm_software.tool_position_velocity_control:main",
            "hotspot_detector.py = robotarm_software.hotspot_detector:main",
            "hotspot_web_stream.py = robotarm_software.hotspot_web_stream:main",
            "go_to_start.py = robotarm_software.go_to_start:main",
            "hotspot_servo.py = robotarm_software.hotspot_servo:main",
        ],
    },
)
