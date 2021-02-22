# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    author="Julian Foerster",
    packages=["highlevel_planning_py"],
    package_dir={"": "../../highlevel_planning/src"},
)

setup(**setup_args)
