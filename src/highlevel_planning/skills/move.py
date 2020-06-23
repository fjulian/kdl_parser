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
    assert np.linalg.norm(direction) - 1.0 < EPS
    projection = np.matmul(direction, direction.T)
    return np.eye(3) - projection


class SkillMove:
    def __init__(self, scene_, robot_, desired_velocity):
        self.scene = scene_
        self.robot = robot_
        self.desired_velocity = desired_velocity

        self.gamma = 0.2

        self.f_desired = np.zeros((3, 1))
        self.alpha_f = 0.2
        self.beta_f = 0.2

        self.dt = 0.01

    def move_object(self, desired_distance, direction_initial_guess, lock=None):

        travelled_distance = 0.0
        direction = np.copy(direction_initial_guess)
        force_integral = np.zeros((3, 1))
        if direction.shape == (3,):
            direction = direction.reshape(3, 1)

        while travelled_distance < desired_distance:
            # Measure force
            f_wristframe = self.robot.get_wrist_force()

            # Compute force reaction (PI controller)
            force_error = f_wristframe - self.f_desired
            projection_matrix = ortho_projection(direction)
            force_integral += self.dt * np.matmul(projection_matrix, force_error)
            v_f = self.alpha_f * force_error + self.beta_f * force_integral

            # Compute new velocity reference
            v_ref = self.desired_velocity * direction - np.matmul(
                projection_matrix, v_f
            )

            # Update direction estimate
            direction += (
                -self.gamma * self.desired_velocity * np.matmul(projection_matrix, v_f)
            )

            # Wait for next step
            time.sleep(0.5)

            # Dummy code for now
            travelled_distance += desired_distance / 5.0

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
