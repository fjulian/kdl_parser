import os
from copy import deepcopy
import pybullet as p
import numpy as np
from math import pi as m_pi
import math
from highlevel_planning.tools.util import IKError, quat_from_mat

from trac_ik_python.trac_ik import IK

from scipy.spatial.transform import Rotation as R

from urdf_parser_py.urdf import URDF as urdf_parser
from pykdl_utils.kdl_kinematics import KDLKinematics

from rc.controllers import CartesianVelocityControllerKDL


# ------- Parameters ------------------
# TODO move to config file

max_force_magnitude = 150

# --------------------


def getMotorJointStates(robot):
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
    joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques


class RobotArm:
    def __init__(self, world, robot_model=None):
        self._world = world
        self._model = robot_model
        self.num_joints = 0
        self.joint_idx_arm = [1, 2, 3, 4, 5, 6, 7]
        self.joint_idx_fingers = [0, 0]
        self.joint_idx_hand = 0
        self.arm_base_link_idx = -100
        self.arm_ee_link_idx = -100

        # Set up IK solver
        self.urdf_path = os.path.join(os.getcwd(), "data/models/box_panda_hand_pb.urdf")
        with open(self.urdf_path) as f:
            if f.mode == "r":
                urdf_string = f.read()
        self.ik_solver = IK(
            "panda_link0", "panda_link8", urdf_string=urdf_string, solve_type="Speed"
        )

        # Set up FK solver
        robot_urdf = urdf_parser.from_xml_string(urdf_string)
        self.kdl_kin = KDLKinematics(robot_urdf, "panda_link0", "panda_link8")

        # Specify start command
        self.start_cmd = np.array(
            [0, -m_pi / 4.0, 0, -3.0 * m_pi / 4.0, 0, m_pi / 2.0, m_pi / 4.0]
        )
        self.start_pos, self.start_orient = self.fk(self.start_cmd)
        # check_cmd = self.ik(self.start_pos, self.start_orient)
        # print("Arm start pose: " + str(self.start_pos) + " " + str(self.start_orient))

        # Standard velocity used for following trajectories
        self.std_vel = 0.3
        self.std_duration = 4

        # Set up velocity setting for driving
        self._world.velocity_setter = self.velocity_setter
        self.velocity_trans = [0.0, 0.0, 0.0]
        self.velocity_turn = 0.0

    def reset(self):
        if self._model is None:
            self._model = self._world.add_model(
                path=self.urdf_path,
                position=[0.0, 0.0, 0.04],
                orientation=[0.0, 0.0, 0.0, 1.0],
            )

        self.link_name_to_index = {
            p.getBodyInfo(self._model.uid)[0]: -1,
        }

        self.num_joints = p.getNumJoints(self._model.uid)
        for i in range(self.num_joints):
            info = p.getJointInfo(self._model.uid, i)
            joint_name = info[1]
            # print(joint_name, info[16])  # Use this to print all joint names.
            if "panda_joint" in joint_name and len(joint_name) == 12:
                joint_num = int(joint_name.split("panda_joint")[1])
                if joint_num < 8:
                    self.joint_idx_arm[joint_num - 1] = i
                if joint_num == 1:
                    # Save the index of the arm base link
                    self.arm_base_link_idx = info[16]
            elif "panda_hand_joint" in joint_name:
                self.arm_ee_link_idx = info[16]
                self.joint_idx_hand = i
            elif "panda_finger_joint" in joint_name:
                joint_num = int(joint_name.split("panda_finger_joint")[1])
                self.joint_idx_fingers[joint_num - 1] = i

            _name = info[12]
            self.link_name_to_index[_name] = i

        p.enableJointForceTorqueSensor(
            self._model.uid, self.joint_idx_hand, enableSensor=1
        )

        self.apply_colors()

    def apply_colors(self):
        rgba_white = [0.9, 0.9, 0.9, 1.0]
        rgba_light_gray = [0.4, 0.4, 0.4, 1.0]
        rgba_black = [0.15, 0.15, 0.15, 1.0]

        use_white = True
        for i in range(1, 8):
            self.apply_color(
                "panda_link{}".format(i), rgba_white if use_white else rgba_light_gray,
            )
            use_white = not use_white

        self.apply_color("panda_hand", rgba_white)
        self.apply_color("panda_rightfinger", rgba_black)
        self.apply_color("panda_leftfinger", rgba_black)

    def apply_color(self, link_name, rgba):
        link_idx = self.link_name_to_index[link_name]
        p.changeVisualShape(
            self._model.uid, linkIndex=link_idx, rgbaColor=rgba,
        )

    def set_joints(self, desired):
        if desired is None:
            return
        p.setJointMotorControlArray(
            self._model.uid,
            self.joint_idx_arm,
            p.POSITION_CONTROL,
            targetPositions=desired,
        )

    def transition_cmd_to(self, desired, duration=None, stop_on_contact=False):
        desired_pos, _ = self.fk(desired)

        current_cmd = np.array(self.get_joints())
        current_pos, _ = self.fk(current_cmd)

        if duration is None:
            duration = np.linalg.norm(current_pos - desired_pos) / self.std_vel

        if duration < 1e-3:
            duration = 0.1

        if duration > self._world.T_s:
            diff = (desired - current_cmd) / float(duration * self._world.f_s)
            for i in range(1, int(math.ceil(duration * self._world.f_s))):
                cmd = current_cmd + i * diff
                self.set_joints(cmd.tolist())
                self._world.step_one()
                self._world.sleep(self._world.T_s)
                if stop_on_contact and not self.check_max_contact_force_ok():
                    return False
        self.set_joints(desired.tolist())
        return True

    def transition_cartesian(
        self, pos_des, orient_des, duration=None, stop_on_contact=False
    ):
        orient_des_rot = R.from_quat(orient_des)
        pos_ee = pos_des - np.matmul(
            orient_des_rot.as_dcm(), np.array([0.0, 0.0, 0.103])
        )

        current_cmd = np.array(self.get_joints())
        current_pos, current_orient = self.fk(current_cmd)

        if duration is None:
            duration = np.linalg.norm(current_pos - pos_ee) / self.std_vel

        if duration < 1e-3:
            duration = 0.1

        diff_pos = (pos_ee - current_pos) / float(duration * self._world.f_s)
        diff_orient = (orient_des - current_orient) / float(duration * self._world.f_s)
        fail_count = 0
        for i in range(1, int(math.ceil(duration * self._world.f_s))):
            pos = current_pos + i * diff_pos
            orient = current_orient + i * diff_orient
            cmd = self.ik(pos, orient, current_cmd)
            if cmd.tolist() is None or cmd is None:
                fail_count += 1
                if fail_count > 10:
                    raise IKError
                continue
            else:
                fail_count = 0
            current_cmd = cmd
            self.set_joints(cmd.tolist())
            self._world.step_one()
            self._world.sleep(self._world.T_s)
            if stop_on_contact and not self.check_max_contact_force_ok():
                return False
        cmd = self.ik(pos_ee, orient_des, current_cmd)
        self.set_joints(cmd.tolist())
        return True

    def transition_function(self, fcn, t_fin):
        current_cmd = np.array(self.get_joints())
        t = 0
        fail_count = 0
        while t < t_fin:
            pos, orient = fcn(t)
            cmd = self.ik(pos, orient, current_cmd)
            if np.any(np.equal(cmd, None)) or cmd is None or cmd.tolist() is None:
                # print("No IK solution found...")
                fail_count += 1
                if fail_count > 10:
                    raise IKError
                continue
            else:
                fail_count = 0
            current_cmd = cmd
            self.set_joints(cmd.tolist())
            self._world.step_one()
            self._world.sleep(self._world.T_s)
            t += self._world.T_s
        pos, orient = fcn(t_fin)
        cmd = self.ik(pos, orient, current_cmd)
        self.set_joints(cmd.tolist())

    def task_space_velocity_control(
        self, velocity_translation, velocity_rotation, num_steps
    ):
        """
        Takes a desired end-effector velocity, computes necessary joint velocities and applies them.
        Needs to be called at every time step.

        Args:
            velocity ([type]): [description]
        """

        ctrl = CartesianVelocityControllerKDL()
        ctrl.init_from_urdf_file(self.urdf_path, "panda_link0", "panda_hand")

        for _ in range(num_steps):
            mpos, _, _ = getMotorJointStates(self._model.uid)

            # Convert velocity from hand frame to base frame
            link_poses = p.getLinkStates(
                self._model.uid,
                linkIndices=[
                    self.arm_base_link_idx,
                    self.link_name_to_index["panda_hand"],
                ],
            )
            base_r = R.from_quat(link_poses[0][1])
            ee_r = R.from_quat(link_poses[1][1])

            velocity_translation_baseframe = base_r.inv().apply(
                ee_r.apply(velocity_translation)
            )
            velocity_rotation_baseframe = base_r.inv().apply(
                ee_r.apply(velocity_rotation)
            )

            cmd = ctrl.compute_command(
                velocity_translation_baseframe, velocity_rotation_baseframe, mpos[0:7]
            )

            # Apply them
            p.setJointMotorControlArray(
                self._model.uid,
                self.joint_idx_arm,
                p.VELOCITY_CONTROL,
                targetVelocities=list(cmd),
            )

            self._world.step_one()
            self._world.sleep(self._world.T_s)

        # Stop the arm
        p.setJointMotorControlArray(
            self._model.uid,
            self.joint_idx_arm,
            p.VELOCITY_CONTROL,
            targetVelocities=[0.0] * len(cmd),
        )

    def check_max_contact_force_ok(self):
        force, _ = self.get_wrist_force_torque()
        magnitude = np.linalg.norm(force)
        if magnitude > max_force_magnitude:
            return False
        else:
            return True

    def get_joints(self):
        temp = p.getJointStates(self._model.uid, self.joint_idx_arm)
        pos = [a[0] for a in temp]
        return pos

    def open_gripper(self):
        pos = [0.038, 0.038]
        p.setJointMotorControlArray(
            self._model.uid,
            self.joint_idx_fingers,
            p.POSITION_CONTROL,
            targetPositions=pos,
        )

    def close_gripper(self):
        pos = [0.0, 0.0]
        forces = [25.0, 25.0]
        p.setJointMotorControlArray(
            self._model.uid,
            self.joint_idx_fingers,
            p.POSITION_CONTROL,
            targetPositions=pos,
            forces=forces,
        )

    def check_grasp(self):
        gripper_state = p.getJointStates(self._model.uid, self.joint_idx_fingers)
        assert len(gripper_state) == 2

        # dist_threshold = 0.01
        # dist = gripper_state[0][0] + gripper_state[1][0]
        # if dist > dist_threshold:
        #     object_present = True
        # else:
        #     object_present = False

        force_threshold = 1.0
        force1 = gripper_state[0][3]
        force2 = gripper_state[1][3]
        if abs(force1) < force_threshold and abs(force2) < force_threshold:
            object_present = False
        else:
            object_present = True

        return object_present

    def ik(self, pos, orient, seed_state=None):
        # Ignore passed in seed state and just query the joints.
        if self._model is None:
            seed_state = [0.0] * self.ik_solver.number_of_joints
        else:
            seed_state = self.get_joints()
        orient = orient / np.linalg.norm(orient)
        sol = self.ik_solver.get_ik(
            seed_state,
            pos[0],
            pos[1],
            pos[2],
            orient[0],
            orient[1],
            orient[2],
            orient[3],
        )
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

    def update_velocity(self, vel_trans, vel_rot):
        # vel_trans and vel_rot are expected to be in robot body frame.
        self.velocity_trans = vel_trans
        self.velocity_turn = vel_rot

    def stop_driving(self):
        self.velocity_trans = [0.0, 0.0, 0.0]
        self.velocity_turn = 0.0

    def velocity_setter(self):
        # Determine current robot pose
        _, orient = p.getBasePositionAndOrientation(self._model.uid)
        orient = R.from_quat(orient)
        # euler = orient.as_euler('xyz', degrees=True)

        # Convert velocity commands to world frame
        vel_trans_world = orient.apply(self.velocity_trans)
        # vel_rot doesn't need to be converted, since body and world z axis coincide.

        p.resetBaseVelocity(
            self._model.uid, vel_trans_world.tolist(), [0.0, 0.0, self.velocity_turn]
        )

    def get_wrist_force_torque(self):
        _, _, f_t, _ = p.getJointState(self._model.uid, self.joint_idx_hand)
        forces = np.array(f_t[:3])
        torques = np.array(f_t[3:])
        return forces, torques
