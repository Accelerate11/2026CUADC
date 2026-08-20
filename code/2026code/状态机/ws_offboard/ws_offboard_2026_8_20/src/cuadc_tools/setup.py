from setuptools import find_packages, setup

package_name = "cuadc_tools"

setup(
    name=package_name,
    version="4.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="CUADC Flight Team",
    maintainer_email="flight-team@example.invalid",
    description="Offline CUADC log analyzer.",
    license="LicenseRef-Proprietary",
    entry_points={
        "console_scripts": [
            "analyze_flight_log = cuadc_tools.analyze_flight_log:main",
        ],
    },
)
