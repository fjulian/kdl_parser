import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import py_trees

import multiprocessing, atexit, time


class ActionNavigate(py_trees.behaviour.Behaviour):
    def __init__(self, process_pipe, target_name, name="nav_action"):
        super(ActionNavigate, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target_name = target_name
        self._process_pipe = process_pipe

    def initialise(self):
        self.logger.debug(
            "%s.initialise()->sending new goal" % (self.__class__.__name__)
        )

        # Empty existing messages from pipe
        while self._process_pipe.poll():
            _ = self._process_pipe.recv()

        # Send command
        self._process_pipe.send([self._target_name])

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        self.feedback_message = "Nav in progress"
        if self._process_pipe.poll():
            res = self._process_pipe.recv().pop()
            if res == 0:
                new_status = py_trees.common.Status.SUCCESS
                self.feedback_message = "Nav successful"
            elif res == 1:
                # Nav in progress, but this is already set above
                pass
            elif res == 2:
                new_status = py_trees.common.Status.FAILURE
                self.feedback_message = "Nav failed"
            else:
                assert (False, "Unexpected response")
        self.logger.debug(
            "%s.update()[%s->%s][%s]"
            % (self.__class__.__name__, self.status, new_status, self.feedback_message)
        )
        return new_status


class ProcessNavigate:
    def __init__(self, scene, robot_uid):
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.nav = multiprocessing.Process(
            target=nav_process, args=(self.child_connection, scene, robot_uid)
        )
        atexit.register(self.nav.terminate)
        self.nav.start()
        print("Navigation process initiated")

    def get_pipe(self):
        return self.parent_connection


def nav_process(pipe_connection, scene, robot_uid):
    sk_nav = SkillNavigate(robot_uid, scene)
    while True:
        if pipe_connection.poll():
            cmd = pipe_connection.recv()
            if len(cmd) == 1:
                res = sk_nav.move_to_object(cmd[0])
                if res:
                    pipe_connection.send([0])
                else:
                    pipe_connection.send([2])
            else:
                print("Unexpected command")
        time.sleep(0.5)


class SkillNavigate:
    def __init__(self, robot_uid, scene):
        self.robot_uid_ = robot_uid
        self.scene_ = scene

    def _check_collisions(self):
        for _, obj in self.scene_.objects.items():
            temp = p.getClosestPoints(self.robot_uid_, obj.model.uid, distance=0.5)
            for elem in temp:
                contact_distance = elem[8]
                if contact_distance < 0.0:
                    # print("There is a collision")
                    return True
        return False

    def _move(self, pos, orient):
        p.resetBasePositionAndOrientation(
            self.robot_uid_, pos.tolist(), orient.tolist()
        )

    def move_to_object(self, target_name):
        target_id = self.scene_.objects[target_name].model.uid

        # Get the object position
        temp = p.getBasePositionAndOrientation(target_id)
        target_pos = np.array(temp[0])

        # Get valid nav angles
        nav_angle = self.scene_.objects[target_name].nav_angle
        nav_min_dist = self.scene_.objects[target_name].nav_min_dist

        # Move there
        return self.move_to_pos(target_pos, nav_angle, nav_min_dist)

    def move_to_pos(self, target_pos, nav_angle=None, nav_min_dist=None):
        # Get robot position
        temp = p.getBasePositionAndOrientation(self.robot_uid_)
        robot_pos = np.array(temp[0])

        if nav_angle is None:
            alphas = np.arange(0.0, 2.0 * np.pi, 2.0 * np.pi / 10.0)
        else:
            alphas = np.array([nav_angle])
        if nav_min_dist is None:
            radii = np.arange(0.4, 2.0, 0.1)
        else:
            radii = nav_min_dist + np.arange(0.4, 2.0, 0.1)

        # Iterate through points on circles around the target
        # First vary the radius
        for r in radii:
            # Then vary the angle
            for alpha in alphas:
                direction_vec = np.array([np.cos(alpha), np.sin(alpha), 0])
                robot_pos[:2] = target_pos[:2] + r * direction_vec[:2]
                rotation = R.from_euler("z", np.pi + alpha, degrees=False)
                robot_orient = rotation.as_quat()

                # Put robot into this position
                self._move(robot_pos, robot_orient)
                if not self._check_collisions():
                    return True
        return False


def get_nav_description():
    action_name = "nav"
    action_params = [["obj", "object"], ["rob", "chimera"]]
    action_preconditions = []
    action_effects = [("in-reach", False, ["obj", "rob"])]
    return (
        action_name,
        {
            "params": action_params,
            "preconds": action_preconditions,
            "effects": action_effects,
        },
    )
