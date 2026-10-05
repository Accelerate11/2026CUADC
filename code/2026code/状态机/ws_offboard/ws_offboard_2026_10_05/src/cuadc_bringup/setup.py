from glob import glob
from setuptools import find_packages, setup

package_name = "cuadc_bringup"
setup(
    name=package_name,
    version="4.0.1",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="CUADC Flight Team",
    maintainer_email="flight-team@example.invalid",
    description="CUADC launch, mission configuration and route heading loader.",
    license="AGPL-3.0-only",
    entry_points={"console_scripts": []},
)
