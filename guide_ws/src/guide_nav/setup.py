from setuptools import find_packages, setup

package_name = "guide_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@example.com",
    description="导览导航服务端：NavigateTo Action",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigate_to_server = guide_nav.navigate_to_server:main",
        ],
    },
)
