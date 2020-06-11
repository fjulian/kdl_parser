
## Todo

- High priority
  - [ ] Write code that generates problem definition based on observations from the simulation
  - [ ] Use python logger instead of print statements.
  - [ ] Try other IK library
- Low priority
  - [ ] Write tests for robot arm control code.
- Done (new on top)
  - [x] Define predicates
  - [x] Define scene for symbol learning (table, cup, box, cupboard with drawers, ...)
  - [x] Implement grasping skill
  - [x] Try closing the whole loop with a trivial example
  - [x] Instances of the robot arm class are distributed over multiple threads/processes. Therefore, variables are not carried over, which breaks the robot's behavior in some cases. Need to find a solution to have the robot class in a single thread only OR make the class state-less.
  - [x] Restore simulation initial state if the simulation is already running, instead of reloading everything. This takes quite some time. Could be triggered via a command line argument or via querying which objects are present in the current simulation.
  - [x] Add argparser
  - [x] Look again at check_grasp function. Seems like it would also say that an object is grasped if the gripper is just open.
  - [x] Change the robot arm transition. Doesn't necessarily need to be cartesian in all cases. Maybe this reduces the twitching.
  - [x] Add navigation to the whole system
  - [x] Find a better grasping pose for the drawer handle.
  - [x] Make releasing an object a separate action
  - [x] Allow robot base velocity command to be given in robot base frame


## Setup Instructions

### Set up virtualenv

```
virtualenv -p /usr/bin/python2 --system-site-packages .venv
source .venv/bin/activate
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
cd <repo-root>/submodules/hrl-kdl/pykdl_utils
python setup.py install --prefix <repo-root>/.venv
cd <repo-root>/submodules/hrl-kdl/hrl_geom
python setup.py install --prefix <repo-root>/.venv
```

### Install Development Code

The library that comes with this package needs to be installed in the virtual environment. To do this in developer mode, use the command

```
pip install -e .
```

in the root directory of the project.

To install them regularly, just for using them, use the command without the flag `-e`.

### Robot Description

The file data/ridgeback_panda_hand.urdf can be generated using the command

```
xacro mopa_description/robots/ridgeback_panda_hand.urdf.xacro
```

### Physics simulator

There are two options: using the physics client GUI that comes with pybullet or launching the server separately and connecting to it via shared memory.

For the first option, no additional setup needs to be done after installing pybullet in the virtual environment.

For the second option, you need to clone and build bullet3 [according to the instructions](https://github.com/bulletphysics/bullet3). It boils down to simply executing the shell script `./build_cmake_pybullet_double.sh` which comes with the repo. Afterwards, the binary is located in the directory `bullet3/build_cmake/examples/ExampleBrowser`. When launching it for the first time, the correct mode "Physics Server" needs to be selected in the left pane. At subsequent launches, this mode will be automatically selected (it gets saved to the file `0_Bullet3Demo.txt`, located in the same folder as the binary).

Depending on which option is selected, the initialization of the simulator interface needs to be adapted in the file `src/sim/world.py`.

### Symbolic Planner

TODO Write this!

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
            ("<predicate1>", <true/false>, ["<param1>", "<param2>", ...]),
        	("<predicate2>", <true/false>, ["<param8>", "<param6>", ...]),
            ...
        ],
        "effects": [
            ("<predicate1>", <true/false>, ["<param1>", "<param2>", ...]),
        	("<predicate7>", <true/false>, ["<param2>", "<param4>", ...]),
            ...
        ]
    },
    "<action2>": ...
}
```

The variable `<true/false>` is true if the predicate must hold before the action can be run (for preconditions) or does hold after the action was run (for effects).

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

### Internal Representations

**Objects**

```python
knowledge_base.objects = {
    "<object_label1>": ["<base_type1>", "<type2>"],
    "<object_label2>": ["<base_type2>", "<type3>"],
}
```


### Interface to External Planner

The external planner can be called from a terminal using the following command:

```bash
./bin/ff -s 2 -o knowledge/chimera/domain/200221-154221_domain.pddl -f knowledge/chimera/problem/200221-154221_problem.pddl
```

Replace domain and problem files appropriately.
