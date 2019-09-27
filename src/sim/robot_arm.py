import os
import pybullet as p
import numpy as np
from math import pi as m_pi
import math
from tools.util import IKError, quat_from_mat

from trac_ik_python.trac_ik import IK

from scipy.spatial.transform import Rotation as R

from urdf_parser_py.urdf import URDF as urdf_parser
from pykdl_utils.kdl_kinematics import KDLKinematics


class RobotArm:
    def __init__(self, world):
        self._world = world
        self._model = None
        self.num_joints = 0
        self.joint_idx_arm = [1, 2, 3, 4, 5, 6, 7]
        self.joint_idx_hand = [0, 0]

        # Set up IK solver
        self.urdf_path = os.path.join(os.getcwd(),"data/ridgeback_panda_hand.urdf")
        with open(self.urdf_path) as f:
            if f.mode == 'r':
                urdf_string = f.read()
        self.ik_solver = IK("panda_link0", "panda_link8", urdf_string=urdf_string, solve_type="Speed")

        # Set up FK solver
        robot_urdf = urdf_parser.from_xml_string(urdf_string)
        self.kdl_kin = KDLKinematics(robot_urdf, "panda_link0", "panda_link8")

        # # Specify start_pos and start_orient
        # self.start_pos = np.array([0.3, 0.0, 0.6])
        # r1 = R.from_euler('z', -45, degrees=True)       # Align the hand to be perpendicular to the x axis
        # r2 = R.from_euler('x', 180, degrees=True)       # Let the hand point downwards
        # r_total = r1*r2
        # self.start_orient = r_total.as_quat()
        # self.start_cmd = self.ik(self.start_pos, self.start_orient)

        # Specify start command
        self.start_cmd = np.array([0, -m_pi/4.0, 0, -3.0*m_pi/4.0, 0, m_pi/2.0, m_pi/4.0])
        self.start_pos, self.start_orient = self.fk(self.start_cmd)
        # check_cmd = self.ik(self.start_pos, self.start_orient)

        # Standard velocity used for following trajectories
        self.std_vel = 0.25
        self.std_duration = 4

        # The robot is loaded in this position initially
        self.current_cmd = np.array([0.0] * 7)
        self.current_pos = np.array([0.116133, 0.0, 0.931720])
        self.current_orient = np.array([-0.82533172, 0.56462609, -0.0041196189, 0.002819567719])
        self.current_pos_updated = True

    def reset(self):
        self._model = self._world.add_model(
            path=self.urdf_path,
            position=[0.0, 0.0, 0.05],
            orientation=[0.0, 0.0, 0.0, 1.0]
        )
        self.num_joints = p.getNumJoints(self._model.uid)

        for i in range(self.num_joints):
            info = p.getJointInfo(self._model.uid, i)
            joint_name = info[1]
            # print(joint_name)  # Use this to print all joint names.
            if "panda_joint" in joint_name and len(joint_name) == 12:
                joint_num = int(joint_name.split("panda_joint")[1])
                if joint_num < 8:
                    self.joint_idx_arm[joint_num-1] = i
            elif "panda_finger_joint" in joint_name:
                joint_num = int(joint_name.split("panda_finger_joint")[1])
                self.joint_idx_hand[joint_num-1] = i

    def set_joints(self, desired):
        if desired is None:
            return
        p.setJointMotorControlArray(self._model.uid, self.joint_idx_arm, p.POSITION_CONTROL, targetPositions=desired)

    def transition_cmd_to(self, desired, duration=None):
        desired_pos, desired_orient = self.fk(desired)
        if duration is None:
            if self.current_pos_updated:
                duration = np.linalg.norm(self.current_pos - desired_pos) / self.std_vel
            else:
                print("WARNING: Current position not up-to-date. Using standard duration.")
                duration = self.std_duration

        try:
            _ = desired - self.current_cmd
        except TypeError as e:
            if not (self.current_cmd is None or self.current_cmd.tolist() is None):
                # print("Unexpected None!!")
                print(desired)
                print(desired is None)
                raise e
        if self.current_cmd.tolist() is None or self.current_cmd is None:
            # TODO figure out why current_cmd is still sometimes None...
            self.current_cmd = self.get_joints()
        if duration > self._world.T_s:
            diff = (desired - self.current_cmd) / float(duration * self._world.f_s)
            for i in range(1, int(math.ceil(duration * self._world.f_s))):
                cmd = self.current_cmd + i*diff
                self.set_joints(cmd.tolist())
                self._world.step_one()
                self._world.sleep(self._world.T_s)
        self.set_joints(desired.tolist())
        self.current_cmd = desired
        self.current_pos = desired_pos
        self.current_orient = desired_orient
        self.current_pos_updated = True

    def transition_cartesian(self, pos_des, orient_des, duration=None):
        if duration is None:
            if self.current_pos_updated:
                duration = np.linalg.norm(self.current_pos - pos_des) / self.std_vel
            else:
                print("WARNING: Current position not up-to-date. Using standard duration.")
                duration = self.std_duration

        diff_pos = (pos_des - self.current_pos) / float(duration * self._world.f_s)
        diff_orient = (orient_des - self.current_orient) / float(duration * self._world.f_s)
        fail_count = 0
        for i in range(1, int(math.ceil(duration * self._world.f_s))):
            pos = self.current_pos + i*diff_pos
            orient = self.current_orient + i*diff_orient
            cmd = self.ik(pos, orient, self.current_cmd)
            if cmd.tolist() is None or cmd is None:
                fail_count += 1
                if fail_count > 10:
                    raise IKError
                continue
            else:
                fail_count = 0
            self.current_cmd = cmd
            self.set_joints(cmd.tolist())
            self._world.step_one()
            self._world.sleep(self._world.T_s)
        cmd = self.ik(pos_des, orient_des, self.current_cmd)
        self.current_cmd = cmd
        self.set_joints(cmd.tolist())
        self.current_pos = pos_des
        self.current_orient = orient_des
        self.current_pos_updated = True

    def transition_function(self, fcn, t_fin):
        t = 0
        fail_count = 0
        while t < t_fin:
            pos, orient = fcn(t)
            cmd = self.ik(pos, orient, self.current_cmd)
            if np.any(np.equal(cmd, None)) or cmd is None or cmd.tolist() is None:
                # print("No IK solution found...")
                fail_count += 1
                if fail_count > 10:
                    raise IKError
                continue
            else:
                fail_count = 0
            self.current_cmd = cmd
            self.set_joints(cmd.tolist())
            self._world.step_one()
            self._world.sleep(self._world.T_s)
            t += self._world.T_s
        pos, orient = fcn(t_fin)
        cmd = self.ik(pos, orient, self.current_cmd)
        self.current_cmd = cmd
        self.current_pos = pos
        self.current_pos_updated = True
        self.current_orient = orient
        self.set_joints(cmd.tolist())

    def get_joints(self):
        temp = p.getJointStates(self._model.uid, self.joint_idx_arm)
        pos = [a[0] for a in temp]
        return pos

    def open_gripper(self):
        pos = [0.038, 0.038]
        p.setJointMotorControlArray(self._model.uid, self.joint_idx_hand, p.POSITION_CONTROL, targetPositions=pos)

    def close_gripper(self):
        pos = [0.0, 0.0]
        forces = [1.5, 1.5]
        p.setJointMotorControlArray(self._model.uid, self.joint_idx_hand, p.POSITION_CONTROL, targetPositions=pos, forces=forces)

    def check_grasp(self):
        gripper_state = p.getJointStates(self._model.uid, self.joint_idx_hand)
        threshold = 0.01
        # print(gripper_state[0][0])
        # print(gripper_state[1][0])

        dist = gripper_state[0][0] + gripper_state[1][0]
        if dist > threshold:
            object_present = True
        else:
            object_present = False
        return object_present

    def ik(self, pos, orient, seed_state=None):
        # if seed_state is None:
        #     seed_state = [0.0] * self.ik_solver.number_of_joints
        # elif seed_state.tolist() is None:
        #     seed_state = [0.0] * self.ik_solver.number_of_joints

        # Ignore passed in seed state and just query the joints.
        if self._model is None:
            seed_state = [0.0] * self.ik_solver.number_of_joints
        else:
            seed_state = self.get_joints()
        orient = orient / np.linalg.norm(orient)
        sol = self.ik_solver.get_ik(seed_state, pos[0], pos[1], pos[2], orient[0], orient[1], orient[2], orient[3])
        return np.array(sol)

    def fk(self, joint_states):
        # Inspired by https://answers.ros.org/question/281272/get-forward-kinematics-wo-tf-service-call-from-urdf-joint-angles-kinetic-python/
        pose = self.kdl_kin.forward(joint_states.tolist())
        pose = np.array(pose)
        transl = pose[:3, 3]
        rot_mat = pose[:3, :3]
        orient = quat_from_mat(rot_mat)
        return transl, orient

    def to_start(self):
        self.transition_cmd_to(self.start_cmd)
