import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import homogenous_trafo, invert_hom_trafo
import py_trees.common
import multiprocessing
import time
import atexit

from highlevel_planning.tools.util import IKError

EPS = 1e-6


def ortho_projection(direction):
    assert np.abs(np.linalg.norm(direction) - 1.0) < EPS
    projection = np.matmul(direction, direction.T)
    return np.eye(3) - projection


class SkillMove:
    def __init__(self, scene_, robot_, desired_velocity, time_step):
        self.scene = scene_
        self.robot = robot_
        self.desired_velocity = desired_velocity

        self.gamma_direction = 0.2
        self.gamma_hinge = 0.2

        self.f_desired = np.zeros((3, 1))
        self.k_p_f = 0.2
        self.k_i_f = 0.2

        self.t_desired = np.zeros((3, 1))
        self.k_p_t = 0.2
        self.k_i_t = 0.2

        self.dt = time_step

        self.cart_vel_ctrl = None

    def move_object(self, desired_distance, direction_initial_guess):

        travelled_distance = 0.0

        direction = np.copy(direction_initial_guess)
        direction /= np.linalg.norm(direction_initial_guess)
        direction = direction.reshape(3, 1)

        hinge_vector = np.zeros((3, 1))

        force_integral = np.zeros((3, 1))
        torque_integral = np.zeros((3, 1))

        while travelled_distance < desired_distance:
            # Measure force and torque
            f_wristframe, t_wristframe = self.robot.get_wrist_force_torque()
            f_wristframe = f_wristframe.reshape(3, 1)
            t_wristframe = t_wristframe.reshape(3, 1)

            # ---- Tranlation -----

            # Compute force reaction (PI controller)
            force_error = f_wristframe - self.f_desired
            projection_matrix = ortho_projection(direction)
            force_integral += self.dt * np.matmul(projection_matrix, force_error)
            v_f = self.k_p_f * force_error + self.k_i_f * force_integral

            # Update direction estimate
            direction -= (
                self.dt
                * self.gamma_direction
                * self.desired_velocity
                * np.matmul(projection_matrix, v_f)
            )

            # Compute new translation velocity reference
            velocity_translation = self.desired_velocity * direction - np.matmul(
                projection_matrix, v_f
            )

            # ---- Rotation -----

            # Torque reaction
            torque_error = t_wristframe - self.t_desired
            torque_integral += self.dt * torque_error
            w_t = self.k_i_t * torque_error + self.k_i_t * torque_integral

            # Update hinge estimate
            hinge_vector -= self.dt * self.gamma_hinge * self.desired_velocity * w_t

            # Compute new rotation velocity reference
            velocity_rotation = self.desired_velocity * hinge_vector - w_t

            # Apply for one step
            self.robot.task_space_velocity_control(
                np.squeeze(velocity_translation), np.squeeze(velocity_rotation), 1
            )

        return True


# def get_move_description():
#     action_name = "move"
#     action_params = [
#         ["obj", "item"],
#         ["pos", "position"],
#         ["rob", "robot"]
#     ]
#     action_preconditions = [
#         ("in-reach-pos", False, ["pos", "rob"]),
#         ("empty-hand", True, ["rob"]),
#         ("in-hand", False, ["obj", "rob"])
#     ]
#     action_effects = [
#         ("empty-hand", False, ["rob"]),
#         ("in-hand", True, ["obj", "rob"])
#     ]
#     return (action_name, {"params": action_params, "preconds": action_preconditions, "effects": action_effects})
