from glob import glob

from setuptools import setup

package_name = "lds07rr_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ArnodeParno",
    description="ROS 2 driver for the Roborock LDS07RR lidar on a Raspberry Pi",
    license="MIT",
    entry_points={"console_scripts": [f"lds07rr_node = {package_name}.node:main"]},
)
