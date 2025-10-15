from setuptools import setup

package_name = "ros2_client"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/traffic_light_client.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autonomous-driving-project",
    maintainer_email="user@example.com",
    description="ROS 2 node that subscribes to IoT server data over TCP and publishes ROS 2 traffic light color updates.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "traffic_light_client = ros2_client.tcp_client_node:main",
        ],
    },
)
