from glob import glob
from setuptools import setup

package_name = "omnibot_nav2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OmniBot Maintainer",
    maintainer_email="robot@example.invalid",
    description="ROS 2 Nav2 integration for OmniBot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "backend_bridge = omnibot_nav2.backend_bridge:main",
        ],
    },
)
