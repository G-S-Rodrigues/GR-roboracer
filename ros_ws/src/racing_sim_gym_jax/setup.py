"""Build metadata for the racing_sim_gym_jax ROS package."""

import os
from pathlib import Path

from setuptools import find_packages, setup

package_name = "racing_sim_gym_jax"
package_root = Path(__file__).resolve().parent


def find_source_root(start):
    """Find the repository root without assuming where setup.py is run from.

    A fixed number of parent hops is wrong the moment this file is read from
    somewhere other than the source tree: colcon reads it from
    <base>/build/racing_sim_gym_jax/, three parents above which is the
    directory holding the build base, not the repository. Inside a
    /ws/.scratch/<worktree> that is /ws/.scratch, so every worktree installs
    the same shared /ws/.scratch/config -- missing, or another branch's
    (repo gotcha #20). Walking up to the directory that actually contains
    this package and a config/ lands on the repository from either location.
    """
    for candidate in [start, *start.parents]:
        if (candidate / "config").is_dir() and (
            candidate / "ros_ws" / "src" / package_name
        ).is_dir():
            return candidate
    raise RuntimeError(
        "cannot locate the repository root (a directory holding config/ and "
        f"ros_ws/src/{package_name}) above {start}"
    )


source_root = find_source_root(package_root)
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
