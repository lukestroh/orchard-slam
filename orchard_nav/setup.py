from setuptools import find_packages, setup
import os
import glob

package_name = "orchard_nav"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob.glob("params/*.yaml")),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your-email@example.com",
    description="Navigation package for orchard environment",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "robot_behavior = orchard_nav.robot_behavior:main",
        ],
    },
)