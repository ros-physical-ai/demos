from glob import glob

from setuptools import find_packages, setup

package_name = "pai_aic"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config/rosetta", glob("config/rosetta/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franco Cipollone",
    maintainer_email="franco.cipollone@ekumenlabs.com",
    description="Record, train and deploy on the AIC cable-insertion simulator with rosetta.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "teleop = pai_aic.teleop:main",
        ],
    },
)
