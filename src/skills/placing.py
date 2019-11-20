import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from tools.util import homogenous_trafo, invert_hom_trafo
import py_trees.common
import multiprocessing
import time
import atexit
import threading

from tools.util import IKError


# class ActionPlacing(py_trees.behaviour.Behaviour):
#     """
#         Based on the example https://py-trees.readthedocs.io/en/release-0.6.x/_modules/py_trees/demos/action.html#Action
#     """
#     def __init__(self, scene, robot, lock, target, name="grasping_action"):
#         super(ActionPlacing, self).__init__(name)
#         self.logger.debug("%s.__init__()" % (self.__class__.__name__))
#         self.setup_called = False
#         self._scene = scene
#         self._robot = robot
#         self._lock = lock
#         self._target = target

#     def setup(self, unused_timeout=15):
#         if not self.setup_called:
#             self.logger.debug("%s.setup()->connections to an external process" % (self.__class__.__name__))
#             self.parent_connection, self.child_connection = multiprocessing.Pipe()
#             self.grasping = multiprocessing.Process(target=grasping_process, args=(self.child_connection, self._scene, self._robot, self._lock))
#             atexit.register(self.grasping.terminate)
#             self.grasping.start()
#             self.setup_called = True
#         return True

#     def initialise(self):
#         self.logger.debug("%s.initialise()->sending new goal" % (self.__class__.__name__))
#         if not self.setup_called:
#             raise RuntimeError("Setup function not called")
#         target_name = self._target[0]
#         target_link_id = self._target[1]
#         target_grasp_id = self._target[2]
#         self.parent_connection.send([target_name, target_link_id, target_grasp_id])

#     def update(self):
#         new_status = py_trees.common.Status.RUNNING
#         self.feedback_message = "Grasping in progress"
#         if self.parent_connection.poll():
#             res = self.parent_connection.recv().pop()
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
#         self.parent_connection.send([])
#         self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


# def grasping_process(pipe_connection, scene, robot, lock):
#     sk_grasping = SkillGrasping(scene, robot)

#     idle = True
#     proc = None
#     try:
#         while True:
#             if pipe_connection.poll():
#                 cmd = pipe_connection.recv()
#                 if len(cmd)==3 and idle:
#                     # Start the process
#                     proc = multiprocessing.Process(target=sk_grasping.grasp_object, args=(cmd[0], cmd[1], cmd[2], lock))
#                     idle = False
#                     proc.start()
#                 elif len(cmd)==0:
#                     # Abort process
#                     if proc:
#                         proc.terminate()
#                     proc = None
#                     idle = True
#             elif not idle and proc.is_alive():
#                 pipe_connection.send([1])
#             elif not idle and not proc.is_alive():
#                 # The thread was launched at some point and seems to be finished now
#                 proc.terminate()
#                 proc = None
#                 idle = True
#                 pipe_connection.send([0])
#             time.sleep(0.5)
#     except KeyboardInterrupt:
#         if proc is not None:
#             proc.terminate()


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
        temp1 = p.getLinkState(self.robot._model.uid, self.robot.arm_base_link_idx)
        r_O_O_rob = np.array(temp1[4]).reshape((-1,1))
        C_O_rob = R.from_quat(np.array(temp1[5]))
        T_O_rob = homogenous_trafo(r_O_O_rob, C_O_rob)
        T_rob_O = invert_hom_trafo(T_O_rob)

        # Pos in robot frame
        r_O_O_obj = target_pos
        r_R_R_obj = np.matmul(T_rob_O, np.append(r_O_O_obj, 1.0).reshape((-1,1)))

        # Orientation in robot frame
        C_rob_ee = R.from_quat(self.robot.start_orient)

        # ----- Place object -------

        pos = np.squeeze(r_R_R_obj[:3,:])
        orient = C_rob_ee

        # Move to pre-place-pose
        pos_pre = pos - np.matmul(orient.as_dcm(), np.array([0.0,0.0,0.15]))
        pos_pre_joints = self.robot.ik(pos_pre, orient.as_quat())
        if pos_pre_joints.tolist() is None:
            if lock is not None:
                lock.release()
            return False
        self.robot.transition_cmd_to(pos_pre_joints)

        try:
            # Go to place pose
            self.robot.transition_cartesian(pos, orient.as_quat())

            self.robot._world.step_seconds(0.2)
            self.robot.open_gripper()
            self.robot._world.step_seconds(0.4)

            # Go back to pre-place-pose
            self.robot.transition_cartesian(pos_pre, orient.as_quat())
        except IKError:
            if lock is not None:
                lock.release()
            return False

        if lock is not None:
            lock.release()
        return True


# def get_placing_description():
#     action_name = "place"
#     action_params = [
#         ["obj", "object"],
#         ["rob", "chimera"]
#     ]
#     action_preconditions = [
#         ("in-reach", False, ["obj", "rob"]),
#         ("empty-hand", False, ["rob"])
#     ]
#     action_effects = [
#         ("empty-hand", True, ["rob"]),
#         ("in-hand", False, ["obj", "rob"])
#     ]
#     return (action_name, {"params": action_params, "preconds": action_preconditions, "effects": action_effects})
