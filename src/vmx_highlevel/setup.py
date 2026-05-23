from glob import glob
import os

from setuptools import find_packages, setup

package_name = "vmx_highlevel"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="VMX Direct Low-Level Maintainers",
    maintainer_email="maintainers@example.com",
    description="PC-side ROS 2 high-level bridge for VMX direct UDP low-level motor and sensor telemetry.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vmx_cmd_vel_mux_node = vmx_highlevel.cmd_vel_mux_node:main",
            "vmx_highlevel_node = vmx_highlevel.diff_drive_highlevel_node:main",
            "vmx_udp_bridge_node = vmx_highlevel.udp_bridge_node:main",
        ],
    },
)
