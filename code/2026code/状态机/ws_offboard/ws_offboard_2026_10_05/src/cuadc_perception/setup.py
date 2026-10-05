from glob import glob
from setuptools import find_packages, setup

package_name = "cuadc_perception"
setup(
    name=package_name,
    version="4.0.1",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/models", glob("models/*")),
        ("share/" + package_name, ["LICENSES.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="CUADC Flight Team",
    maintainer_email="flight-team@example.invalid",
    description="Original D435i basket vision, ONNX and segmentation helpers.",
    license="AGPL-3.0-only",
    entry_points={"console_scripts": ['basket_vision_ros_node = cuadc_perception.basket_vision_ros_node:main', 'basket_detect_seg_analysis = cuadc_perception.basket_detect_seg_analysis:main', 'preflight_camera_view = cuadc_perception.preflight_camera_view:main']},
)
