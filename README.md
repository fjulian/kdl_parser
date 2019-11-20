
## Todo

- High priority
  - [ ] Define predicates
  - [ ] Make releasing an object a separate action
  - [ ] Write code that generates problem definition based on observations from the simulation
  - [ ] Use python logger instead of print statements.
  - [ ] Find a better grasping pose for the drawer handle.
  - [ ] Try other IK library
- Low priority
  - [ ] Write tests for robot arm control code.
- Done
  - [x] Define scene for symbol learning (table, cup, box, cupboard with drawers, ...)
  - [x] Implement grasping skill
  - [x] Try closing the whole loop with a trivial example
  - [x] Instances of the robot arm class are distributed over multiple threads/processes. Therefore, variables are not carried over, which breaks the robot's behavior in some cases. Need to find a solution to have the robot class in a single thread only OR make the class state-less.
  - [x] Restore simulation initial state if the simulation is already running, instead of reloading everything. This takes quite some time. Could be triggered via a command line argument or via querying which objects are present in the current simulation.
  - [x] Add argparser
  - [x] Look again at check_grasp function. Seems like it would also say that an object is grasped if the gripper is just open.
  - [x] Change the robot arm transition. Doesn't necessarily need to be cartesian in all cases. Maybe this reduces the twitching.
  - [x] Add navigation to the whole system


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

The file `src/pddl_interface/pddl_file_if.py` contains the code to parse PDDL files and fill the definitions into python data structures, safe and load the python data structures (using pickle), and write modified data structures to new PDDL files. In the following, the layout of the relevant Python data structures is defined. 

**Predicates**

```python
{
    "<predicate1>": [["<param1>", "<type1>"], ["<param2>", "<type2>"], ...],
    "<predicate2>": [...],
    ...
}
```

**Actions**

```python
{
    "<action1>":
    {
        "params": [["<param1>", "<type1>"], ["<param2>", "<type2>"], ...],
        "preconds": [
            ("<predicate1>", <negated>, ["<param1>", "<param2>", ...]),
        	("<predicate2>", <negated>, ["<param8>", "<param6>", ...]),
            ...
        ],
        "effects": [
            ("<predicate1>", <negated>, ["<param1>", "<param2>", ...]),
        	("<predicate7>", <negated>, ["<param2>", "<param4>", ...]),
            ...
        ]
    },
    "<action2>": ...
}
```

The variable `<negated>` is true if the predict mustn't hold (for preconditions) or doesn't hold (for effects, after applying them).

**Objects**

```python
[
    ("<object_label1>", "<type1>"),
    ("<object_label2>", "<type2>"),
    ("<object_label3>", "<type3>")
]
```

**Initial Predicates and Goals**

```python
[
    ("<predicate1>", "<param1>", "<param2>", ...),
    ("<predicate2>", "<param1>", "<param2>", ...),
    ("<predicate3>", "<param1>", "<param2>", ...)
]
```
