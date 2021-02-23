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
    packages=[
        package_name,
        "highlevel_planning_py.execution",
        "highlevel_planning_py.exploration",
        "highlevel_planning_py.knowledge",
        "highlevel_planning_py.pddl_interface",
        "highlevel_planning_py.predicate_learning",
        "highlevel_planning_py.sim",
        "highlevel_planning_py.skills",
        "highlevel_planning_py.tools",
    ],
    package_dir={
        "highlevel_planning_py": "../../highlevel_planning/src/highlevel_planning_py",
        package_name: package_name,
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + "highlevel_planning_py"],
        ),
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
    entry_points={
        "console_scripts": [
            "run_server = highlevel_planning_ros2.run_predicate_learning:main",
            "run_gui = highlevel_planning_ros2.gui:main",
        ]
    },
)
