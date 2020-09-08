import sys
from os import path, getcwd, makedirs

from highlevel_planning.sim.world import World
from highlevel_planning.sim.robot_arm import RobotArm
from highlevel_planning.sim.scene_planning_1 import ScenePlanning1

import pybullet as p

import pickle

BASEDIR = path.dirname(path.dirname(path.abspath(__file__)))

if __name__ == "__main__":
    restore_existing_objects = False
    objects = None
    robot_mdl = None
    if restore_existing_objects:
        with open("data/sim/objects.pkl", "rb") as pkl_file:
            objects, robot_mdl = pickle.load(pkl_file)

    # Create world
    world = World(
        style="shared", sleep_=True, load_objects=not restore_existing_objects
    )
    scene = ScenePlanning1(world, BASEDIR, restored_objects=objects)

    # Spawn robot
    robot = RobotArm(world, base_dir=BASEDIR, robot_model=robot_mdl)
    robot.reset()

    robot.to_start()
    world.step_seconds(1)

    # Save
    savedir = "data/sim"
    if not path.isdir(savedir):
        makedirs(savedir)
    with open("data/sim/objects.pkl", "wb") as output:
        pickle.dump((scene.objects, robot._model), output)
    p.saveBullet(path.join(getcwd(), "data/sim/state.bullet"))
