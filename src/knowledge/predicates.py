from skills.grasping import SkillGrasping
import pybullet as p


class Predicates:
    def __init__(self, scene, robot, robot_lock):
        self.call = {
            "empty-hand": self.empty_hand,
            "in-hand": self.in_hand,
            "in-reach": self.in_reach,
            "in-reach-pos": self.in_reach_pos
        }

        self.descriptions = {
            "empty-hand": [["rob", "chimera"]],
            "in-hand": [["obj", "object"], ["rob", "chimera"]],
            "in-reach": [["obj", "object"], ["rob", "chimera"]],
            "in-reach-pos": [["pos", "position"], ["rob", "chimera"]]
        }

        self.sk_grasping = SkillGrasping(scene, robot)
        self._scene = scene
        self._robot_uid = robot._model.uid
        self._robot_lock = robot_lock

    def empty_hand(self, robot):
        self._robot_lock.acquire()
        grasped_sth = robot.check_grasp()
        self._robot_lock.release()
        return not grasped_sth

    def in_hand(self, target_object, robot):
        empty_hand_res = self.empty_hand(robot)
        temp = p.getClosestPoints(self._robot_uid, self._scene.objects[target_object].model.uid, distance=0.01)
        dist_finger1 = 100
        dist_finger2 = 100
        for contact in temp:
            if contact[3] == robot.joint_idx_hand[0]:
                dist_finger1 = contact[8]
            elif contact[3] == robot.joint_idx_hand[1]:
                dist_finger2 = contact[8]
        desired_object_in_hand = (abs(dist_finger1)<0.001) and (abs(dist_finger2)<0.001)
        return ( (not empty_hand_res) and desired_object_in_hand )
        
    def in_reach(self, target_object, robot):
        self._robot_lock.acquire()
        pos, orient = self.sk_grasping.compute_grasp(target_object, None, 0)
        cmd = robot.ik(pos, orient)
        self._robot_lock.release()
        if cmd.tolist() is None or cmd is None:
            return False
        else:
            return True

    def in_reach_pos(self, target_pos, robot):
        self._robot_lock.acquire()
        cmd = robot.ik(target_pos, robot.start_orient)
        self._robot_lock.release()
        if cmd.tolist() is None or cmd is None:
            return False
        else:
            return True
