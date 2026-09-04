from setuptools import find_packages, setup

package_name = "vision_servo_calibration"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/alignment.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="maintainer@example.invalid",
    description="Detector-agnostic payload alignment crosshair and calibration tools.",
    license="LicenseRef-Choose-Before-Publishing",
    entry_points={
        "console_scripts": [
            "alignment_viewer = vision_servo_calibration.alignment_viewer_node:main",
            "vision_provider_template = vision_servo_calibration.vision_provider_template:main",
            "vision_contract_check = vision_servo_calibration.vision_contract_check:main",
        ],
    },
)
