import os
from glob import glob
from setuptools import find_packages, setup

package_name = "vio_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="VIO Team",
    maintainer_email="tu.nguyendevwork@hcmut.edu.vn",
    description="ROS 2 wrapper for the MSCKF VIO pipeline",
    license="MIT",
    entry_points={
        "console_scripts": [
            "py_sub = vio_pkg.py_sub:main",
            "bag_reader = vio_pkg.bag_reader:main",
            "vio_node = vio_pkg.vio_node:main",
            "vio_system_node = vio_pkg.vio_node:main",
        ],
    },
)
