"""注册 ROS 2 雷达包、资源索引和可执行入口。"""

from setuptools import setup

setup(
    name="delta2c_lidar",
    version="0.1.0",
    packages=["delta2c_lidar"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/delta2c_lidar"]),
        ("share/delta2c_lidar", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xu",
    maintainer_email="xu@example.com",
    description="DELTA-2C PRO ROS 2 serial driver",
    license="MIT",
    entry_points={"console_scripts": ["lidar_node = delta2c_lidar.node:main"]},
)
