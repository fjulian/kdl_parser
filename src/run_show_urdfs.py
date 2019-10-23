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

    # ======== Cupboard ====================================

    # cupboard_mdl = world.add_model("data/cupboard_drawers/cupboard_drawers.urdf", position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])

    # drawer_link_idx = []
    # for i in range(p.getNumJoints(cupboard_mdl.uid)):
    #     info = p.getJointInfo(cupboard_mdl.uid, i)
    #     joint_name = info[1]
    #     if "drawer_joint" in joint_name and len(joint_name) == 13:
    #         drawer_link_idx.append(i)

    # for i in drawer_link_idx:
    #     p.setJointMotorControl2(cupboard_mdl.uid, i, controlMode=p.VELOCITY_CONTROL, force=0.0)

    # ======== Container ======================================

    container_mdl = world.add_model("data/container/container_no_lid.urdf", position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])
    lid_mdl = world.add_model("data/container/lid.urdf", position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])

    container_sliding_mdl = world.add_model("data/container/container_sliding_lid.urdf", position=[0.0, 0.5, 0.0], orientation=[0.0, 0.0, 0.0,1.0])

    # =========================================================

    world.step_seconds(50)

    world.close()

if __name__ == "__main__":
    main()
