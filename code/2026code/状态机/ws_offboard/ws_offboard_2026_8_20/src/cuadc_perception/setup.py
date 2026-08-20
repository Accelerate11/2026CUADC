from glob import glob
from setuptools import find_packages, setup

package_name = "cuadc_perception"

setup(
    name=package_name,
    version="4.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSES.md"]),
        ("share/" + package_name + "/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="CUADC Flight Team",
    maintainer_email="flight-team@example.invalid",
    description="Basket segmentation from aligned RealSense ROS image topics.",
    license="LicenseRef-Proprietary AND AGPL-3.0-only",
    entry_points={
        "console_scripts": [
            "bucket_perception_node = cuadc_perception.bucket_perception_node:main",
        ],
    },
)
