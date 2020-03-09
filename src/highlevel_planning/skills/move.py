import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import homogenous_trafo, invert_hom_trafo
import py_trees.common
import multiprocessing
import time
import atexit

from highlevel_planning.tools.util import IKError


# class ActionPlacing(py_trees.behaviour.Behaviour):
#     """
#     Based on the example https://py-trees.readthedocs.io/en/release-0.6.x/_modules/py_trees/demos/action.html#Action
#     """
#     def __init__(self, process_pipe, target_pos, name="placing_action"):
#         super(ActionPlacing, self).__init__(name)
#         self.logger.debug("%s.__init__()" % (self.__class__.__name__))
#         self._target_pos = target_pos
#         self._process_pipe = process_pipe

#     def initialise(self):
#         self.logger.debug("%s.initialise()->sending new goal" % (self.__class__.__name__))
#         self._process_pipe.send([self._target_pos])

#     def update(self):
#         new_status = py_trees.common.Status.RUNNING
#         self.feedback_message = "Grasping in progress"
#         if self._process_pipe.poll():
#             res = self._process_pipe.recv().pop()
#             if res==0:
#                 new_status = py_trees.common.Status.SUCCESS
#                 self.feedback_message = "Grasping successful"
#             elif res==1:
#                 # Grasping in progress, but this is already set above
#                 pass
#             elif res==2:
#                 new_status = py_trees.common.Status.FAILURE
#                 self.feedback_message = "Grasping failed"
#             else:
#                 assert(False, "Unexpected response")
#         self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
#         return new_status

#     def terminate(self, new_status):
#         self._process_pipe.send([])
#         self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


# class ProcessPlacing:
#     def __init__(self, scene, robot, robot_lock):
#         self.parent_connection, self.child_connection = multiprocessing.Pipe()
#         self.placing = multiprocessing.Process(target=placing_process, args=(self.child_connection, scene, robot, robot_lock))
#         atexit.register(self.placing.terminate)
#         self.placing.start()
#         print("Placing process initiated")

#     def get_pipe(self):
#         return self.parent_connection


# def placing_process(pipe_connection, scene, robot, lock):
#     sk_placing = SkillPlacing(scene, robot)

#     while True:
#         if pipe_connection.poll():
#             cmd = pipe_connection.recv()
#             if len(cmd)==1:
#                 # Start the process
#                 res = sk_placing.place_object(cmd[0], lock)
#                 if res:
#                     pipe_connection.send([0])
#                 else:
#                     pipe_connection.send([2])

#                 # Clear commands that came in while running this
#                 while pipe_connection.poll():
#                     cmd = pipe_connection.recv()
#                     if len(cmd) > 0:
#                         print("WARNING! Received multiple place commands simultaneously.")
#         time.sleep(0.5)


class SkillMove:
    def __init__(self, scene_, robot_):
        self.scene = scene_
        self.robot = robot_

    def move_object(self, desired_distance, direction_initial_guess, lock=None):

        travelled_distance = 0.0
        while travelled_distance < desired_distance:
            # Measure current state

            # Update belief of constraint using past positions and forces

            # Run MPC step to compute next control inputs

            # Apply first input to base and arm

            # Wait for next step
            time.sleep(0.5)

            # Dummy code for now
            travelled_distance += desired_distance / 5.0

        return True


# def get_move_description():
#     action_name = "place"
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
