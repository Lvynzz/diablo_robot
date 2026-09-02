from glob import glob
import os

from setuptools import find_packages, setup


package_name = "diablo_web_interface"


def collect_data_files(source_dir, install_dir):
    """Return data_files entries while preserving a directory tree."""
    entries = []
    if not os.path.isdir(source_dir):
        return entries

    for root, _dirs, files in os.walk(source_dir):
        paths = [os.path.join(root, name) for name in files]
        if not paths:
            continue
        relative = os.path.relpath(root, source_dir)
        destination = install_dir if relative == "." else os.path.join(install_dir, relative)
        entries.append((destination, paths))
    return entries


data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
]

for directory in ("config", "maps"):
    data_files.extend(
        collect_data_files(
            directory,
            os.path.join("share", package_name, directory),
        )
    )

web_static_source = "dist" if os.path.isdir("dist") else os.path.join(package_name, "static")
data_files.extend(
    collect_data_files(
        web_static_source,
        os.path.join("share", package_name, "static"),
    )
)


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alvin",
    maintainer_email="alvin@todo.todo",
    description="Nav2 and browser interface for the Diablo robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "web_node = diablo_web_interface.web_node:main",
            "motion_cmd_bridge = diablo_web_interface.motion_cmd_bridge:main",
            "motion_cmd_mux = diablo_web_interface.motion_cmd_mux:main",
            "wheel_odom = diablo_web_interface.wheel_odom:main",
        ],
    },
)
