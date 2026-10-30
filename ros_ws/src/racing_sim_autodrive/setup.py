"""Build metadata for the racing_sim_autodrive ROS package."""

from setuptools import find_packages, setup

package_name = "racing_sim_autodrive"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Guilherme Rodrigues",
    maintainer_email="guilherme6821@gmail.com",
    description="ROS-free core of the AutoDRIVE simulator backend.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "racing_sim_autodrive_server = "
            "racing_sim_autodrive.server_node:main",
            "racing_sim_autodrive_replay = racing_sim_autodrive.replay:main",
            "racing_sim_autodrive_devkit_standin = "
            "racing_sim_autodrive.devkit_standin:main",
        ],
    },
)
