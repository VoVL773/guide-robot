from setuptools import setup

package_name = "learn_e2c"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/move_forward.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@example.com",
    description="ROS 2 入门练习包",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "move_forward = learn_e2c.move_forward_node:main",
        ],
    },
)
