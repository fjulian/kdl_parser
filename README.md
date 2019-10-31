
## Todo

- [ ] Define scene for symbol learning (table, cup, box, cupboard with drawers, ...)
- [x] Implement grasping skill
- [ ] Write tests for robot arm control code.


## Setup Instructions

### Set up virtualenv

```
virtualenv -p /usr/bin/python2 --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

Python version 2 is used because trac IK does not work with python 3 yet.

### Dependencies

Some ROS packages need to be installed:

```
sudo apt-get install ros-melodic-trac-ik ros-melodic-franka-description ros-melodic-ridgeback-description
```

In addition, the library `pykdl_utils` - included as submodule - needs to be installed. This can be done using the following commands:

```
cd <repo-root>/src/tools/hrl-kdl/pykdl_utils
python setup.py install --prefix <repo-root>/venv
cd <repo-root>/src/tools/hrl-kdl/hrl_geom
python setup.py install --prefix <repo-root>/venv
```

### Robot Description

The file data/ridgeback_panda_hand.urdf can be generated using the command

```
xacro mopa_description/robots/ridgeback_panda_hand.urdf.xacro
```

## Documentation

### Representation of planning problem in Python


