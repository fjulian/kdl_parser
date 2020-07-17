import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import SkillExecutionError
import py_trees.common
import multiprocessing
import time
import atexit


class ActionGrasping(py_trees.behaviour.Behaviour):
    """
        Based on the example https://py-trees.readthedocs.io/en/release-0.6.x/_modules/py_trees/demos/action.html#Action
    """

    def __init__(self, process_pipe, target, name="grasping_action"):
        super(ActionGrasping, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target = target
        self._process_pipe = process_pipe

    def initialise(self):
        self.logger.debug(
            "%s.initialise()->sending new goal" % (self.__class__.__name__)
        )

        # Empty existing messages from pipe
        while self._process_pipe.poll():
            _ = self._process_pipe.recv()

        # Send command
        target_name = self._target[0]
        target_link_id = self._target[1]
        target_grasp_id = self._target[2]
        self._process_pipe.send([target_name, target_link_id, target_grasp_id])

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        self.feedback_message = "Grasping in progress"
        if self._process_pipe.poll():
            res = self._process_pipe.recv().pop()
            if res == 0:
                new_status = py_trees.common.Status.SUCCESS
                self.feedback_message = "Grasping successful"
            elif res == 1:
                # Grasping in progress, but this is already set above
                pass
            elif res == 2:
                new_status = py_trees.common.Status.FAILURE
                self.feedback_message = "Grasping failed"
            else:
                assert (False, "Unexpected response")
        self.logger.debug(
            "%s.update()[%s->%s][%s]"
            % (self.__class__.__name__, self.status, new_status, self.feedback_message)
        )
        return new_status

    def terminate(self, new_status):
        self._process_pipe.send([])
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )


class ProcessGrasping:
    def __init__(self, scene, robot, robot_lock):
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.grasping = multiprocessing.Process(
            target=grasping_process,
            args=(self.child_connection, scene, robot, robot_lock),
        )
        atexit.register(self.grasping.terminate)
        self.grasping.start()
        print("Grasping process initiated")

    def get_pipe(self):
        return self.parent_connection


def grasping_process(pipe_connection, scene, robot, lock):
    sk_grasping = SkillGrasping(scene, robot)

    idle = True
    proc = None
    try:
        while True:
            if pipe_connection.poll():
                cmd = pipe_connection.recv()
                if len(cmd) == 3 and idle:
                    # Start the process
                    proc = multiprocessing.Process(
                        target=sk_grasping.grasp_object,
                        args=(cmd[0], cmd[1], cmd[2], lock),
                    )
                    idle = False
                    proc.start()
                elif len(cmd) == 0:
                    # Abort process
                    if proc:
                        proc.terminate()
                    proc = None
                    idle = True
            elif not idle and proc.is_alive():
                pipe_connection.send([1])
            elif not idle and not proc.is_alive():
                # The thread was launched at some point and seems to be finished now
                proc.terminate()
                proc = None
                idle = True
                pipe_connection.send([0])
            time.sleep(0.5)
    except KeyboardInterrupt:
        if proc is not None:
            proc.terminate()


class SkillGrasping:
    def __init__(self, scene_, robot_):
        self.scene = scene_
        self.robot = robot_

        self.last_pre_pos = None
        self.last_pre_orient = None

    def compute_grasp(self, target_name, link_idx=0, grasp_id=0):
        obj_info = self.scene.objects[target_name]
        target_id = obj_info.model.uid
        link_id = obj_info.grasp_links[link_idx]

        num_grasps = len(obj_info.grasp_pos[link_id])
        if num_grasps == 0:
            raise SkillExecutionError("No grasps defined for this object")
        if grasp_id >= num_grasps:
            raise SkillExecutionError("Invalid grasp ID")

        # Get the object pose
        if link_id == -1:
            temp = p.getBasePositionAndOrientation(target_id)
            r_O_O_obj = np.array(temp[0]).reshape((-1, 1))
            C_O_obj = R.from_quat(np.array(temp[1]))
        else:
            temp = p.getLinkState(target_id, link_id)
            r_O_O_obj = np.array(temp[4]).reshape((-1, 1))
            C_O_obj = R.from_quat(np.array(temp[5]))

        # Get grasp data
        r_Obj_obj_grasp = obj_info.grasp_pos[link_id][grasp_id].reshape((-1, 1))

        # Get robot arm base orientation
        temp1 = p.getLinkState(self.robot._model.uid, self.robot.arm_base_link_idx)
        C_O_rob = R.from_quat(np.array(temp1[5]))

        # Compute desired position of end effector in robot frame
        r_O_O_grasp = r_O_O_obj + C_O_obj.apply(r_Obj_obj_grasp.squeeze()).reshape(
            (-1, 1)
        )
        r_R_R_grasp = self.robot.convert_pos_to_robot_frame(r_O_O_grasp)

        # self.robot._world.draw_cross(np.squeeze(r_O_O_grasp))

        # Compute desired orientation
        C_obj_grasp = R.from_quat(obj_info.grasp_orient[link_id][grasp_id])
        C_rob_ee_default = R.from_quat(self.robot.start_orient)
        C_rob_grasp = C_O_rob.inv() * C_O_obj * C_obj_grasp
        C_rob_ee = (
            C_rob_grasp * C_rob_ee_default
        )  # Apply standard EE orientation. EE will be in default orientation if robot and grasp orientation are equal

        return r_R_R_grasp[:3], C_rob_ee.as_quat()

    def grasp_object(self, target_name, link_idx=0, grasp_id=0, lock=None):
        if lock is not None:
            lock.acquire()
        pos, orient = self.compute_grasp(target_name, link_idx, grasp_id)

        self.robot.open_gripper()

        # Go to pre-grasp pose
        pos_pre = pos - np.matmul(
            R.from_quat(orient).as_dcm(),
            np.array([0.0, 0.0, 0.15]),  # TODO move to config file
        )
        pos_pre_joints = self.robot.ik(pos_pre, orient)
        if pos_pre_joints.tolist() is None:
            if lock is not None:
                lock.release()
            return False
        self.robot.transition_cmd_to(pos_pre_joints)
        self.robot._world.step_seconds(0.5)

        # Go to grasp pose
        self.robot.transition_cartesian(pos, orient)

        self.robot._world.step_seconds(0.2)
        self.robot.close_gripper()
        self.robot._world.step_seconds(0.4)

        # Save some variables required for releasing
        self.last_pre_pos = pos_pre
        self.last_pre_orient = orient

        if lock is not None:
            lock.release()
        return True

    def release_object(self):
        pos_current, orient_current = self.robot.fk(np.array(self.robot.get_joints()))
        pos_retract = pos_current - np.matmul(
            R.from_quat(orient_current).as_dcm(), np.array([0.0, 0.0, 0.07])
        )

        self.robot.open_gripper()
        self.robot._world.step_seconds(0.2)
        self.robot.transition_cartesian(pos_retract, orient_current)


def get_grasping_description():
    action_name = "grasp"
    action_params = [["obj", "item"], ["rob", "robot"]]
    action_preconditions = [
        ("in-reach", True, ["obj", "rob"]),
        ("empty-hand", True, ["rob"]),
    ]
    action_effects = [("empty-hand", False, ["rob"]), ("in-hand", True, ["obj", "rob"])]
    return (
        action_name,
        {
            "params": action_params,
            "preconds": action_preconditions,
            "effects": action_effects,
        },
    )
