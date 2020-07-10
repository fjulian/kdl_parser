import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import py_trees
import multiprocessing, atexit, time

from highlevel_planning.tools.util import (
    homogenous_trafo,
    invert_hom_trafo,
    pos_and_orient_from_hom_trafo,
)


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
                raise (ValueError, "Unexpected response")
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
    sk_nav = SkillNavigate(scene, robot_uid)
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
    def __init__(self, scene, robot):
        self.robot_ = robot
        self.robot_uid_ = robot._model.uid
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

    def move_to_object(self, target_name, nav_min_dist=None):
        target_id = self.scene_.objects[target_name].model.uid

        # Get the object position
        temp = p.getBasePositionAndOrientation(target_id)
        target_pos = np.array(temp[0])

        # Get valid nav angles
        nav_angle = self.scene_.objects[target_name].nav_angle
        if nav_min_dist is None:
            nav_min_dist = self.scene_.objects[target_name].nav_min_dist

        # Move there
        return self.move_to_pos(target_pos, nav_angle, nav_min_dist)

    def move_to_pos(self, target_pos, nav_angle=None, nav_min_dist=None):
        assert len(target_pos) == 3
        assert type(target_pos) is np.ndarray

        self.robot_.to_start()

        # Get robot position
        temp = p.getBasePositionAndOrientation(self.robot_uid_)
        robot_pos = np.array(temp[0])
        robot_orient = R.from_quat(temp[1])

        # Get position and orientation of any object in the robot hand w.r.t the robot base
        object_in_hand_uid = self._find_object_in_hand()
        T_rob_obj = self._get_object_relative_pose(
            object_in_hand_uid, robot_pos, robot_orient
        )

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
                    # Move object into robot's hand
                    self._set_object_relative_pose(
                        object_in_hand_uid, robot_pos, robot_orient, T_rob_obj
                    )

                    return True
        return False

    def _find_object_in_hand(self):
        # Determine which object is in the robot's hand
        object_in_hand_uid = None
        for _, obj in self.scene_.objects.items():
            temp = p.getClosestPoints(
                self.robot_uid_,
                obj.model.uid,
                distance=0.01,
                linkIndexA=self.robot_.link_name_to_index["panda_leftfinger"],
            )
            if len(temp) > 0:
                if object_in_hand_uid is not None:
                    raise RuntimeError(
                        "Don't know how to deal with more than one object in robot's hand"
                    )
                object_in_hand_uid = obj.model.uid
        return object_in_hand_uid

    def _get_object_relative_pose(self, object_in_hand_uid, robot_pos, robot_orient):
        T_rob_obj = None
        if object_in_hand_uid is not None:
            # Get object position
            temp = p.getBasePositionAndOrientation(object_in_hand_uid)
            held_object_pos = np.array(temp[0])
            held_object_orient = R.from_quat(temp[1])

            # Compute object pose relative to robot
            r_O_O_obj = held_object_pos
            C_O_obj = held_object_orient
            T_O_obj = homogenous_trafo(r_O_O_obj, C_O_obj)
            r_O_O_rob = robot_pos
            C_O_rob = robot_orient
            T_O_rob = homogenous_trafo(r_O_O_rob, C_O_rob)

            T_rob_obj = np.matmul(invert_hom_trafo(T_O_rob), T_O_obj)

            # Check result
            T_test = np.matmul(T_O_rob, T_rob_obj)
            assert np.all(T_test - T_O_obj < 1e-12)

        return T_rob_obj

    def _set_object_relative_pose(
            self, object_in_hand_uid, robot_pos, robot_orient, T_rob_obj
    ):
        if object_in_hand_uid is not None:
            r_O_O_rob = robot_pos
            C_O_rob = R.from_quat(robot_orient)
            T_O_rob = homogenous_trafo(r_O_O_rob, C_O_rob)

            T_O_obj = np.matmul(T_O_rob, T_rob_obj)
            (held_object_pos, held_object_orient,) = pos_and_orient_from_hom_trafo(
                T_O_obj
            )
            p.resetBasePositionAndOrientation(
                object_in_hand_uid,
                held_object_pos.tolist(),
                held_object_orient.tolist(),
            )


def get_nav_in_reach_description():
    action_name = "nav-in-reach"
    action_params = [
        ["current_pos", "navgoal"],
        ["goal_pos", "navgoal"],
        ["rob", "robot"],
    ]
    action_preconditions = [("in-reach", True, ["current_pos", "rob"])]
    action_effects = [
        ("in-reach", True, ["goal_pos", "rob"]),
        ("in-reach", False, ["current_pos", "rob"]),
    ]
    return (
        action_name,
        {
            "params": action_params,
            "preconds": action_preconditions,
            "effects": action_effects,
        },
    )


def get_nav_at_description():
    action_name = "nav-at"
    action_params = [
        ["current_pos", "position"],
        ["goal_pos", "position"],
        ["rob", "robot"],
    ]
    action_preconditions = [("at", True, ["current_pos", "rob"])]
    action_effects = [
        ("at", True, ["goal_pos", "rob"]),
        ("at", False, ["current_pos", "rob"]),
    ]
    return (
        action_name,
        {
            "params": action_params,
            "preconds": action_preconditions,
            "effects": action_effects,
        },
    )
