import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from tools.util import homogenous_trafo

class SkillGrasping:
    def __init__(self, scene_, robot_):
        self.scene = scene_
        self.robot = robot_

    def grasp_object(self, target_id):
        # Get the object pose
        temp = p.getBasePositionAndOrientation(target_id)
        obj_pos = np.array(temp[0])
        obj_orient = R.from_quat(np.array(temp[1]))
        
        T_O_obj = homogenous_trafo(obj_pos, obj_orient)

        # Get robot arm base pose
        print("Start pose: "+str(self.robot.start_pos))
        max_val = 10
        max_i=-1
        max_j=-1
        max_type = -1
        for i in range(13):
            for j in range(i+1,13):
                temp1 = p.getLinkState(self.robot._model.uid, i)
                temp2 = p.getLinkState(self.robot._model.uid, j)
                test1 = np.abs(np.array(temp2[0])-np.array(temp1[0]) - self.robot.start_pos)
                if np.max(test1) < max_val:
                    max_val = np.max(test1)
                    max_i = i
                    max_j = j
                    max_type = 1
                test2 = np.abs(np.array(temp2[4])-np.array(temp1[4]) - self.robot.start_pos)
                if np.max(test2) < max_val:
                    max_val = np.max(test2)
                    max_i = i
                    max_j = j
                    max_type = 2
                test2 = np.abs(np.array(temp2[2])-np.array(temp1[2]) - self.robot.start_pos)
                if np.max(test2) < max_val:
                    max_val = np.max(test2)
                    max_i = i
                    max_j = j
                    max_type = 3
        print("Max val: "+str(max_val)+" i: "+str(max_i)+" j: "+str(max_j)+" type: "+str(max_type))
        
        # Compute pos and orient in robot arm frame
