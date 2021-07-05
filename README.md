
## Setup Instructions

Tested on Ubuntu 20.04 with ROS Melodic.

### ROS workspace

This package needs to be built in a ROS workspace. The same workspace should contain ASL's `moma` repository:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:fjulian/asl_highlevel_planning.git
cd asl_highlevel_planning
git submodule update --init
cd ..
git clone git@github.com:ethz-asl/moma.git
cd moma
git submodule update --init
```

From now on, it is assumed, that all commands are run in `~/catkin_ws/src/asl_highlevel_planning`.

### Dependencies

First of all, install some apt packages using

```bash
./install_requirements.sh
```

### Install Development Code

To make the python package available, just build the package:

```
catkin build -DCMAKE_BUILD_TYPE=Release highlevel_planning
```

If an error message comes up when building `trac_ik`, complaining about the header `nlopt.hpp` missing, it helps to install nlopt from source: https://github.com/stevengj/nlopt.

### Set up virtualenv

```
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Physics simulator

There are two options: using the physics client GUI that comes with pybullet or launching the server separately and connecting to it via shared memory.

For the first option (recommended), no additional setup needs to be done after installing pybullet in the virtual environment.

For the second option, you need to clone and build bullet3 [according to the instructions](https://github.com/bulletphysics/bullet3). It boils down to simply executing the shell script `./build_cmake_pybullet_double.sh` which comes with the repo. Afterwards, the binary is located in the directory `bullet3/build_cmake/examples/ExampleBrowser`. When launching it for the first time, the correct mode "Physics Server" needs to be selected in the left pane. At subsequent launches, this mode will be automatically selected (it gets saved to the file `0_Bullet3Demo.txt`, located in the same folder as the binary).

## Run

To run any command, make sure that your ROS distribution, the ROS workspace, as well as the virtual environment are sourced.

### Predicate Learning

In order to run the predicate learning, after launching a ROS master, run the "server" in one terminal:

```bash
python highlevel_planning/scripts/run_predicate_learning.py -m gui -s
```

And the GUI in another:

```bash
python highlevel_planning/scripts/gui.py
```

After starting the simulation (press "Run"), you can rearrange objects using drag and drop. If you want to send a certain arrangement as a training sample, enter predicate name, arguments, and the argument that is to be considered relative (i.e. the one that can be modified to achieve the predicate. Should always be the same.), e.g.:

- Name: `on`
- Arguments: `table,cube1`
- Relative argument: `1`

Once, you're happy with the training examples, press "Build rules" to generate the rules.

Then, after rearranging the objects, you can let the system classify the current situation by pressing "Classify". Monitor the terminal output of the server of the result.

To let the system ask for the label of a certain state of its choosing, press "inquire" while the simulation is running. Afterwards, if the predicate holds in the new state, press "confirm" and otherwise "deny". 

## Other

### Symbolic Planner

TODO Write this!

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
./highlevel_planning/bin/ff -s 2 -o ~/Data/highlevel_planning/knowledge/ScenePlanning1/explore/210410-001641_domain.pddl -f ~/Data/highlevel_planning/knowledge/ScenePlanning1/explore/210410-001641_problem.pddl
```

Replace domain and problem files appropriately.

### Copy code to EULER

```bash
rsync -r --progress --delete --exclude=.venv --exclude=.git --exclude=__pycache__ --exclude=.idea ./asl_highlevel_planning fjulian@euler.ethz.ch:Code/
```
