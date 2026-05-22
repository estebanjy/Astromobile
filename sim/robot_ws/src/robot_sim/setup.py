from setuptools import find_packages, setup
import os
from glob import glob

package_name = "robot_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch",  glob("launch/*.py")),
        (f"share/{package_name}/urdf",    glob("urdf/*")),
        (f"share/{package_name}/worlds",  glob("worlds/*")),
        (f"share/{package_name}/config",  glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Simulacion Gazebo + Nav2 para robot movil con brazo SO101",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "autonomy_bridge = robot_sim.autonomy_bridge:main",
        ],
    },
)
