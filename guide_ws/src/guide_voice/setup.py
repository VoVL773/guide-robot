from glob import glob
import os

from setuptools import find_packages, setup

package_name = "guide_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@example.com",
    description="导览语音服务端：Speak Action -> 平台 TTS",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speak_action_server = guide_voice.speak_action_server:main",
        ],
    },
)
