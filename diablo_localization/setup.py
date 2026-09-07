from glob import glob
import os

from setuptools import setup


package_name = "diablo_localization"


data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
]


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alvin",
    maintainer_email="alvin@todo.todo",
    description="Resettable local wheel odometry for Diablo, with optional legacy EKF",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "local_odom = diablo_localization.local_odom:main",
            "reset_pose = diablo_localization.reset_pose:main",
        ],
    },
)
