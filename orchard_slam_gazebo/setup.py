from setuptools import find_packages, setup

import os
import glob

package_name = "orchard_slam_gazebo"


meshes_relative_base_path = "meshes"
mesh_files = glob.glob("meshes/**/*", recursive=True)
mesh_files = [f for f in mesh_files if os.path.isfile(f)]

_data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob.glob("launch/*.launch.py")),
    # (os.path.join('share', package_name, 'models'), glob.glob('models/*')),
    (os.path.join("share", package_name, "worlds"), glob.glob("worlds/*.sdf")),
]

# Keep file structures for meshes
for file in mesh_files:
    relative_path = os.path.relpath(file, meshes_relative_base_path)
    install_path = os.path.join('share', package_name, meshes_relative_base_path, os.path.dirname(relative_path))
    _data_files.append((install_path, [file]))


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=_data_files,
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lukestroh",
    maintainer_email="luke.strohbehn@gmail.com",
    description="Gazebo package for the orchard slam",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
