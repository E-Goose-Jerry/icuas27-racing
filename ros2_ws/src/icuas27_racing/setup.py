from glob import glob

from setuptools import setup

package_name = "icuas27_racing"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        f"{package_name}.detectors",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="E-Goose-Jerry",
    maintainer_email="duanqinruiwang@gmail.com",
    description="ICUAS 2027 racing: gate detection, race mission, ground-station bridge",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "race_mission = icuas27_racing.race_node:main",
            "gate_detector = icuas27_racing.gate_detector:main",
        ],
    },
)
