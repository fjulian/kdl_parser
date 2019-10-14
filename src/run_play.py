from sim.world import World
from sim.robot_arm import RobotArm

from sim.scene_tossing import SceneTossing
from sim.scene_planning_1 import ScenePlanning1

from skills.navigate import SkillNavigation
from skills.grasping import SkillGrasping

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    # Create world
    world = World(gui_=True, sleep_=True)
    world.add_plane()

    cupboard_mdl = world.add_model("data/cupboard_drawers/cupboard_drawers.urdf", position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])
    # world.add_model("cube_small.urdf", position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])

    drawer_bottom_link_idx = 5
    for i in range(p.getNumJoints(cupboard_mdl.uid)):
        info = p.getJointInfo(cupboard_mdl.uid, i)
        print("================")
        print(info)

    world.forces.append((cupboard_mdl.uid, drawer_bottom_link_idx, [0.0, -10.0, 0.0], [0.0, 0.0, 0.0], p.WORLD_FRAME))

    p.setJointMotorControl2(cupboard_mdl.uid, 5, controlMode=p.TORQUE_CONTROL, force=0.0)

    # raw_input()
    world.step_seconds(50)

    world.close()

if __name__ == "__main__":
    main()
