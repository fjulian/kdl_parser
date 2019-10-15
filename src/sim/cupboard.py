import pybullet as p
from tools.util import ObjectInfo
import numpy as np

class Cupboard:
    def __init__(self, world, pos_, orient_):
        self.urdf = "data/cupboard_drawers/cupboard_drawers.urdf"
        self.model = world.add_model(self.urdf, position=pos_, orientation=orient_)
        self.pos = pos_
        self.orient = orient_
        drawer_link_idx = []
        for i in range(p.getNumJoints(self.model.uid)):
            info = p.getJointInfo(self.model.uid, i)
            joint_name = info[1]
            if "drawer_joint" in joint_name and len(joint_name) == 13:
                drawer_link_idx.append(i)
        for i in drawer_link_idx:
            p.setJointMotorControl2(self.model.uid, i, controlMode=p.VELOCITY_CONTROL, force=0.0)
        
    def get_info(self):
        return ObjectInfo(urdf_path_=self.urdf,
                            init_pos_=np.array(self.pos),
                            init_orient_=np.array(self.orient),
                            grasp_pos_=[
                                np.array([0.0, 0.0, 0.0])
                            ],
                            grasp_orient_=[
                                np.array([0.0, 0.0, 0.0, 1.0])
                            ],
                            model_=self.model
                            )
