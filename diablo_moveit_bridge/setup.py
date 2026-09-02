from glob import glob
from setuptools import setup


package_name = "diablo_moveit_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [
            "resource/" + package_name,
        ]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "ik_moveit_bridge = diablo_moveit_bridge.ik_moveit_bridge:main",
        ],
    },
)
