import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from tools.util import homogenous_trafo, invert_hom_trafo

class SkillGrasping:
    def __init__(self, scene_, robot_):
        self.scene = scene_
        self.robot = robot_

        self.last_pre_pos = None
        self.last_pre_orient = None

    def compute_grasp(self, target_name, link_id):
        obj_info = self.scene.objects[target_name]
        target_id = obj_info.model.uid

        num_grasps = len(obj_info.grasp_pos)
        grasp_id = 0

        # Get the object pose
        if link_id is None:
            temp = p.getBasePositionAndOrientation(target_id)
            r_O_O_obj = np.array(temp[0]).reshape((-1,1))
            C_O_obj = R.from_quat(np.array(temp[1]))
        else:
            temp = p.getLinkState(target_id, link_id)
            r_O_O_obj = np.array(temp[4]).reshape((-1,1))
            C_O_obj = R.from_quat(np.array(temp[5]))

        # Get grasp data
        r_Obj_obj_grasp = obj_info.grasp_pos[grasp_id].reshape((-1,1))

        # Get robot arm base pose
        temp1 = p.getLinkState(self.robot._model.uid, self.robot.arm_base_link_idx)
        # temp2 = p.getLinkState(self.robot._model.uid, self.robot.arm_ee_link_idx)
        r_O_O_rob = np.array(temp1[4]).reshape((-1,1))
        C_O_rob = R.from_quat(np.array(temp1[5]))

        T_O_rob = homogenous_trafo(r_O_O_rob, C_O_rob)
        T_rob_O = invert_hom_trafo(T_O_rob)

        # Compute desired position of end effector in robot frame
        r_O_O_grasp = r_O_O_obj + np.matmul(C_O_obj.as_dcm(),r_Obj_obj_grasp.reshape((-1,1)))
        r_R_R_grasp = np.matmul(T_rob_O, np.append(r_O_O_grasp, 1.0).reshape((-1,1)))
        
        self.robot._world.draw_cross(np.squeeze(r_O_O_grasp))

        # Compute desired orientation
        C_obj_grasp = R.from_quat(obj_info.grasp_orient[grasp_id])
        C_rob_ee = R.from_quat(self.robot.start_orient)
        C_ee_grasp = R.from_dcm(np.matmul(C_O_obj.as_dcm(), np.matmul(C_obj_grasp.as_dcm(), C_O_obj.inv().as_dcm()))) * C_O_obj * C_O_rob.inv()
        C_rob_grasp =  C_ee_grasp * C_rob_ee

        # r_O_O_ee = np.array(temp2[4]).reshape((-1,1))
        # r_Rob_rob_ee = np.matmul(T_rob_O, np.append(r_O_O_ee, 1.0).reshape((-1,1)))
        
        return np.squeeze(r_R_R_grasp[:3,:]), C_rob_grasp.as_quat()
        # return np.squeeze(r_Rob_rob_ee[:3,:]), C_rob_grasp.as_quat()

    def grasp_object(self, target_name, link_id=None):
        pos, orient = self.compute_grasp(target_name, link_id)

        self.robot.open_gripper()

        # Go to pre-grasp pose
        pos_pre = pos - np.matmul(R.from_quat(orient).as_dcm(), np.array([0.0,0.0,0.15]))
        self.robot.transition_cartesian(pos_pre, orient)

        # Go to grasp pose
        self.robot.transition_cartesian(pos, orient)

        self.robot._world.step_seconds(0.2)
        self.robot.close_gripper()
        self.robot._world.step_seconds(0.2)

        # Save some variables required for releasing
        self.last_pre_pos = pos_pre
        self.last_pre_orient = orient

    def release_object(self):
        self.robot.open_gripper()
        self.robot.transition_cartesian(self.last_pre_pos, self.last_pre_orient)
