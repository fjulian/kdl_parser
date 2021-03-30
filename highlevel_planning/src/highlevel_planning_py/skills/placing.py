import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from highlevel_planning_py.tools.util import (
    homogenous_trafo,
    invert_hom_trafo,
    SkillExecutionError,
    IKError,
)


class SkillPlacing:
    """
    The placing skill takes a position in the global frame.
    It moves over the position and then moves downward until a surface is hit.

    Possible extension: yaw angle of the object as input. Or full orientation.
    """

    def __init__(self, scene_, robot_):
        self.scene = scene_
        self.robot = robot_

    def place_object(self, target_pos, lock=None):
        if lock is not None:
            lock.acquire()

        self.robot.to_start()

        # ----- Compute place location in robot frame -------

        # Get robot arm base pose
        temp1 = p.getLinkState(
            self.robot.model.uid,
            self.robot.arm_base_link_idx,
            physicsClientId=self.robot.pb_id,
        )
        r_O_O_rob = np.array(temp1[4]).reshape((-1, 1))
        C_O_rob = R.from_quat(np.array(temp1[5]))
        T_O_rob = homogenous_trafo(r_O_O_rob, C_O_rob)
        T_rob_O = invert_hom_trafo(T_O_rob)

        # Pos in robot frame
        r_O_O_obj = target_pos
        r_R_R_obj = np.matmul(T_rob_O, np.reshape(np.append(r_O_O_obj, 1.0), (-1, 1)))

        # Orientation in robot frame
        C_rob_ee = R.from_quat(self.robot.start_orient)

        # ----- Place object -------

        pos = np.squeeze(r_R_R_obj[:3, :])
        orient = C_rob_ee

        try:
            # Move to pre-place-pose
            pos_pre = pos - np.matmul(orient.as_dcm(), np.array([0.0, 0.0, 0.15]))
            pos_pre_joints = self.robot.ik(pos_pre, orient.as_quat())
            if pos_pre_joints.tolist() is None:
                raise IKError
            collision_during_pre = False
            if not self.robot.transition_cmd_to(pos_pre_joints, stop_on_contact=True):
                collision_during_pre = True

            # Go to place pose
            if not collision_during_pre:
                self.robot.transition_cartesian(
                    pos, orient.as_quat(), stop_on_contact=True
                )

            # Remove grasp constraints
            for constraint in self.robot.grasped_objects:
                p.removeConstraint(
                    userConstraintUniqueId=constraint, physicsClientId=self.robot.pb_id
                )

            self.robot._world.step_seconds(0.2)
            self.robot.open_gripper()
            self.robot._world.step_seconds(0.5)

            # Go back to pre-place-pose
            if not collision_during_pre:
                # self.robot.transition_cartesian(pos_pre, orient.as_quat())
                self.robot.transition_cmd_to(pos_pre_joints)
            else:
                self.robot.to_start()
        except IKError:
            if lock is not None:
                lock.release()
            raise SkillExecutionError

        if lock is not None:
            lock.release()
        return True


def get_placing_description():
    action_name = "place"
    action_params = [["obj", "item"], ["pos", "position"], ["rob", "robot"]]
    action_preconditions = [
        ("in-reach", True, ["pos", "rob"]),
        ("empty-hand", False, ["rob"]),
        ("in-hand", True, ["obj", "rob"]),
    ]
    action_effects = [("empty-hand", True, ["rob"]), ("in-hand", False, ["obj", "rob"])]
    return (
        action_name,
        {
            "params": action_params,
            "preconds": action_preconditions,
            "effects": action_effects,
        },
    )
