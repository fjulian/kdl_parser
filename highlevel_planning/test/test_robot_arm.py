import unittest

import numpy as np
from scipy.spatial.transform import Rotation as R
from copy import deepcopy

import pybullet as p

from highlevel_planning.sim.world import World
from highlevel_planning.sim.robot_arm import RobotArm

TOL = 1e-4


def getHandLinkPos(robot):
    link_pose = p.getLinkState(
        robot._model.uid, linkIndex=robot.link_name_to_index["panda_hand"]
    )
    return np.array(link_pose[0])


def getBaseLinkPos(robot):
    link_pose = p.getLinkState(
        robot._model.uid, linkIndex=robot.link_name_to_index["panda_link0"]
    )
    return np.array(link_pose[0])


def getHandLinkOrient(robot):
    link_pose = p.getLinkState(
        robot._model.uid, linkIndex=robot.link_name_to_index["panda_hand"]
    )
    return R.from_quat(link_pose[1])


class TestCartesianVelocity(unittest.TestCase):
    def setUp(self):
        self.world = World(gui_=False, sleep_=False)
        self.world.add_plane()
        self.robot = RobotArm(self.world)
        self.robot.reset()
        self.robot.to_start()
        self.world.step_seconds(2.5)

    def test_moving(self):
        vel_trans = np.array([0.15, 0.0, 0.0])
        vel_rot = np.array([0.0, 0.0, 0.0])

        hand_rotation_before = getHandLinkOrient(self.robot)
        pos_before_worldframe = getHandLinkPos(self.robot) - getBaseLinkPos(self.robot)
        pos_before_localframe = (0.0, 0.0, 0.0)
        self.robot.task_space_velocity_control(vel_trans, vel_rot, 200)
        hand_rotation_after = getHandLinkOrient(self.robot)
        pos_after_worldframe = getHandLinkPos(self.robot) - getBaseLinkPos(self.robot)
        pos_after_localframe = hand_rotation_before.inv().apply(
            pos_after_worldframe
        ) - hand_rotation_before.inv().apply(pos_before_worldframe)

        self.assertTrue(pos_after_localframe[0] > pos_before_localframe[0])
        self.assertTrue((pos_after_localframe[1] - pos_before_localframe[1]) ** 2 < TOL)
        self.assertTrue((pos_after_localframe[2] - pos_before_localframe[2]) ** 2 < TOL)
        self.assertTrue(
            np.linalg.norm(
                hand_rotation_before.as_quat() - hand_rotation_after.as_quat()
            )
            < 1e-2
        )

        # Move robot to other starting pose
        cmd = deepcopy(self.robot.start_cmd)
        cmd[5] += 0.8
        cmd[0] += 2.0
        self.robot.transition_cmd_to(cmd)

        hand_rotation_before = getHandLinkOrient(self.robot)
        pos_before_worldframe = getHandLinkPos(self.robot) - getBaseLinkPos(self.robot)
        pos_before_localframe = (0.0, 0.0, 0.0)
        self.robot.task_space_velocity_control(vel_trans, vel_rot, 200)
        hand_rotation_after = getHandLinkOrient(self.robot)
        pos_after_worldframe = getHandLinkPos(self.robot) - getBaseLinkPos(self.robot)
        pos_after_localframe = hand_rotation_before.inv().apply(
            pos_after_worldframe
        ) - hand_rotation_before.inv().apply(pos_before_worldframe)

        self.assertTrue(pos_after_localframe[0] > pos_before_localframe[0])
        self.assertTrue((pos_after_localframe[1] - pos_before_localframe[1]) ** 2 < TOL)
        self.assertTrue((pos_after_localframe[2] - pos_before_localframe[2]) ** 2 < TOL)
        self.assertTrue(
            np.linalg.norm(
                hand_rotation_before.as_quat() - hand_rotation_after.as_quat()
            )
            < 1e-2
        )


if __name__ == "__main__":
    unittest.main()
