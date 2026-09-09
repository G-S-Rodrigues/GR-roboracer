"""Build metadata for the racing_sim_gym_jax ROS package."""

import os
from pathlib import Path

from setuptools import find_packages, setup

package_name = "racing_sim_gym_jax"
package_root = Path(__file__).resolve().parent
source_root = Path(__file__).resolve().parents[3]
config_root = source_root / "config"
data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    ("share/" + package_name, ["package.xml"]),
]
for source in sorted(config_root.rglob("*")):
    if source.is_file() and source.name != ".gitkeep":
        destination = (
            Path("share")
            / package_name
            / source.parent.relative_to(source_root)
        )
        data_files.append(
            (str(destination), [os.path.relpath(source, package_root)])
        )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Guilherme Rodrigues",
    maintainer_email="guilherme6821@gmail.com",
    description="ROS adapter for the pinned f1tenth_gym_jax backend.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "racing_sim_gym_jax_node = racing_sim_gym_jax.node:main",
        ],
    },
)
