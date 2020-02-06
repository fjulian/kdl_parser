from skills.grasping import SkillGrasping
import pybullet as p
import numpy as np


class Predicates:
    def __init__(self, scene, robot, robot_lock=None):
        self.call = {
            "empty-hand": self.empty_hand,
            "in-hand": self.in_hand,
            "in-reach": self.in_reach,
            "in-reach-pos": self.in_reach_pos,
            "inside": self.inside,
        }

        self.descriptions = {
            "empty-hand": [["rob", "chimera"]],
            "in-hand": [["obj", "object"], ["rob", "chimera"]],
            "in-reach": [["obj", "object"], ["rob", "chimera"]],
            "in-reach-pos": [["pos", "position"], ["rob", "chimera"]],
            "inside": [["container", "object"], ["contained", "object"]],
        }

        self.sk_grasping = SkillGrasping(scene, robot)
        self._scene = scene
        self._robot_uid = robot._model.uid
        self._robot_lock = robot_lock

    def empty_hand(self, robot):
        if self._robot_lock:
            self._robot_lock.acquire()
        grasped_sth = robot.check_grasp()
        if self._robot_lock:
            self._robot_lock.release()
        return not grasped_sth

    def in_hand(self, target_object, robot):
        empty_hand_res = self.empty_hand(robot)
        temp = p.getClosestPoints(
            self._robot_uid, self._scene.objects[target_object].model.uid, distance=0.01
        )
        dist_finger1 = 100
        dist_finger2 = 100
        for contact in temp:
            if contact[3] == robot.joint_idx_hand[0]:
                dist_finger1 = contact[8]
            elif contact[3] == robot.joint_idx_hand[1]:
                dist_finger2 = contact[8]
        desired_object_in_hand = (abs(dist_finger1) < 0.001) and (
            abs(dist_finger2) < 0.001
        )
        return (not empty_hand_res) and desired_object_in_hand

    def in_reach(self, target_object, robot):
        """
        Check if an object in the scene is in reach of the robot arm.
        
        Arguments:
            target_object (string): The object name, as stated in the scene
                                      file and/or the problem PDDL.
            robot (RobotArm): The interface class of the robot arm to use.
        
        Returns:
            bool: Whether the object can be grasped from the robot's current position.
        """
        if self._robot_lock:
            self._robot_lock.acquire()
        pos, orient = self.sk_grasping.compute_grasp(target_object, None, 0)
        cmd = robot.ik(pos, orient)
        if self._robot_lock:
            self._robot_lock.release()
        if cmd.tolist() is None or cmd is None:
            return False
        else:
            return True

    def in_reach_pos(self, target_pos, robot):
        """
        Similar to function "in_reach", but takes position instead of object name.
        
        Args:
            target_pos (list): 3D vector of position to query
            robot ([type]): [description]
        
        Returns:
            [type]: [description]
        """
        if self._robot_lock:
            self._robot_lock.acquire()
        cmd = robot.ik(target_pos, robot.start_orient)
        if self._robot_lock:
            self._robot_lock.release()
        if cmd.tolist() is None or cmd is None:
            return False
        else:
            return True

    def inside(self, container_object, contained_object):
        """
        Checks if one object in the scene is inside another.
        
        Args:
            container_object (string): Object name of the container object.
            contained_object (string): Object name of the contained object.
        
        Returns:
            bool: Whether the container object contains the contained one.
        """
        container_uid = self._scene.objects[container_object].model.uid
        contained_uid = self._scene.objects[contained_object].model.uid
        aabb_container = p.getAABB(container_uid)
        pos_contained, _ = p.getBasePositionAndOrientation(contained_uid)
        pos_contained = np.array(pos_contained)

        lower_border = np.array(aabb_container[0])
        upper_border = np.array(aabb_container[1])

        return np.all(np.greater_equal(pos_contained, lower_border)) and np.all(
            np.less_equal(pos_contained, upper_border)
        )

    def on(self, supporting_object, supported_object):
        """
        [summary]
        
        Args:
            supporting_object ([type]): [description]
            supported_object ([type]): [description]
        """
        supporting_uid = self._scene.objects[supporting_object].model.uid
        supported_uid = self._scene.objects[supported_object].model.uid
        aabb_supporting = p.getAABB(supporting_uid)

        pos_supported, _ = p.getBasePositionAndOrientation(supported_uid)
        pos_supported = np.array(pos_supported)
        aabb_supported = p.getAABB(supported_uid)

        lower_supporting = np.array(aabb_supporting[0])
        upper_supporting = np.array(aabb_supporting[1])
        lower_supported = np.array(aabb_supported[0])

        # Check if supported object is above supporting one (z-coordinate)
        above_tol = 0.05  # TODO move this to parameter file
        above = lower_supported[2] > upper_supporting[2] - above_tol

        # Check if supported object is within footprint of supporting one (xy-plane).
        # Currently this is based on the position of the supported object. Need to see whether this makes sense.
        within = np.all(
            np.greater_equal(pos_supported[:2], lower_supporting[:2])
        ) and np.all(np.less_equal(pos_supported[:2], upper_supporting[:2]))

        return above and within
