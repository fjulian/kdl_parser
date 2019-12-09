import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from tools.util import homogenous_trafo, invert_hom_trafo
import py_trees.common
import multiprocessing
import time
import atexit

from tools.util import IKError


class ActionPlacing(py_trees.behaviour.Behaviour):
    """
    Based on the example https://py-trees.readthedocs.io/en/release-0.6.x/_modules/py_trees/demos/action.html#Action
    """
    def __init__(self, process_pipe, target_pos, name="placing_action"):
        super(ActionPlacing, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target_pos = target_pos
        self._process_pipe = process_pipe

    def initialise(self):
        self.logger.debug("%s.initialise()->sending new goal" % (self.__class__.__name__))
        self._process_pipe.send([self._target_pos])

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        self.feedback_message = "Grasping in progress"
        if self._process_pipe.poll():
            res = self._process_pipe.recv().pop()
            if res==0:
                new_status = py_trees.common.Status.SUCCESS
                self.feedback_message = "Grasping successful"
            elif res==1:
                # Grasping in progress, but this is already set above
                pass
            elif res==2:
                new_status = py_trees.common.Status.FAILURE
                self.feedback_message = "Grasping failed"
            else:
                raise ValueError("Unexpected response")
        self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        return new_status

    def terminate(self, new_status):
        self._process_pipe.send([])
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class ProcessPlacing:
    def __init__(self, scene, robot, robot_lock):
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.placing = multiprocessing.Process(target=placing_process, args=(self.child_connection, scene, robot, robot_lock))
        atexit.register(self.placing.terminate)
        self.placing.start()
        print("Placing process initiated")
    
    def get_pipe(self):
        return self.parent_connection


def placing_process(pipe_connection, scene, robot, lock):
    sk_placing = SkillPlacing(scene, robot)

    while True:
        if pipe_connection.poll():
            cmd = pipe_connection.recv()
            if len(cmd)==1:
                # Start the process
                res = sk_placing.place_object(cmd[0], lock)
                if res:
                    pipe_connection.send([0])
                else:
                    pipe_connection.send([2])

                # Clear commands that came in while running this
                while pipe_connection.poll():
                    cmd = pipe_connection.recv()
                    if len(cmd) > 0:
                        print("WARNING! Received multiple place commands simultaneously.")
        time.sleep(0.5)


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

        try:
            # Move to pre-place-pose
            pos_pre = pos - np.matmul(orient.as_dcm(), np.array([0.0,0.0,0.15]))
            pos_pre_joints = self.robot.ik(pos_pre, orient.as_quat())
            if pos_pre_joints.tolist() is None:
                raise IKError
            self.robot.transition_cmd_to(pos_pre_joints)
        
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


def get_placing_description():
    action_name = "place"
    action_params = [
        ["obj", "object"],
        ["pos", "position"],
        ["rob", "chimera"]
    ]
    action_preconditions = [
        ("in-reach-pos", False, ["pos", "rob"]),
        ("empty-hand", True, ["rob"]),
        ("in-hand", False, ["obj", "rob"])
    ]
    action_effects = [
        ("empty-hand", False, ["rob"]),
        ("in-hand", True, ["obj", "rob"])
    ]
    return (action_name, {"params": action_params, "preconds": action_preconditions, "effects": action_effects})
