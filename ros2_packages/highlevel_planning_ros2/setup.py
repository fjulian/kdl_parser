from setuptools import setup

try:
    import ament_index_python
except ModuleNotFoundError:
    print("Not a ROS 2 environment. Skipping this one.")
    exit()

package_name = "highlevel_planning_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=["highlevel_planning_py", package_name],
    package_dir={"": "../../highlevel_planning/src", package_name: package_name},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fjulian",
    maintainer_email="julian.foerster@mavt.ethz.ch",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
