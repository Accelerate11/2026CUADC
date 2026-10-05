from glob import glob
from setuptools import find_packages, setup

package_name = "cuadc_tools"
setup(
    name=package_name,
    version="4.0.1",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),

    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="CUADC Flight Team",
    maintainer_email="flight-team@example.invalid",
    description="CUADC route calibration, MAVLink stream setup and telemetry recorder.",
    license="AGPL-3.0-only",
    entry_points={"console_scripts": ['calibrate_route = cuadc_tools.calibrate_route:main', 'flight_telemetry_recorder = cuadc_tools.flight_telemetry_recorder:main', 'mavlink_stream_configurator = cuadc_tools.mavlink_stream_configurator:main']},
)
