import sys
from os import path, getcwd

sys.path.append(path.join(path.dirname(path.dirname(path.abspath(__file__))), "src"))

from highlevel_planning.sim.world import World
from highlevel_planning.sim.robot_arm import RobotArm
from highlevel_planning.sim.scene_planning_1 import ScenePlanning1

import pybullet as p

import pickle

if __name__ == "__main__":
    restore_existing_objects = False
    objects = None
    robot_mdl = None
    if restore_existing_objects:
        with open("data/sim/objects.pkl", "rb") as pkl_file:
            objects, robot_mdl = pickle.load(pkl_file)

    # Create world
    world = World(gui_=True, sleep_=True, load_objects=not restore_existing_objects)
    scene = ScenePlanning1(world, restored_objects=objects)

    # Spawn robot
    robot = RobotArm(world, robot_mdl)
    robot.reset()

    robot.to_start()
    world.step_seconds(1)

    # Save
    with open("data/sim/objects.pkl", "wb") as output:
        pickle.dump((scene.objects, robot._model), output)
    p.saveBullet(path.join(getcwd(), "data/sim/state.bullet"))
