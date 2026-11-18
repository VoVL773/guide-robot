from setuptools import setup

package_name = "guide_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/voice_chat.launch.py"]),
        ("share/" + package_name + "/config", ["config/places.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@example.com",
    description="展厅导览机器人：语音对话（本地千问）与按点位导航",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "voice_chat = guide_voice.voice_chat_node:main",
        ],
    },
)
