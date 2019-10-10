import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R

class SkillNavigation:
    def __init__(self, scene_, robot_uid_):
        self.scene = scene_
        self.robot_uid = robot_uid_

    def check_collisions(self):
        for key, obj in self.scene.objects.items():
            temp = p.getClosestPoints(self.robot_uid, obj.model.uid, distance=0.5)
            for elem in temp:
                contact_distance = elem[8]
                if contact_distance < 0.0:
                    # print("There is a collision")
                    return True
            # res = p.getContactPoints(self.robot_uid, obj.model.uid)
            # if len(res) > 0:
            #     print("There is a collision!")
            #     return True
            # else:
            #     pass
            #     # TODO Make sure that shortest distance is also above some threshold
        return False

    def move(self, pos, orient):
        p.resetBasePositionAndOrientation(self.robot_uid, pos.tolist(), orient.tolist())
        # p.stepSimulation()

    def move_to_object(self, target_id):
        # Get the object position
        temp = p.getBasePositionAndOrientation(target_id)
        target_pos = np.array(temp[0])

        temp = p.getBasePositionAndOrientation(self.robot_uid)
        robot_pos = np.array(temp[0])

        # Iterate through points on circles around the target
        # First vary the radius
        for r in np.arange(0.4, 1.5, 0.1):
            # Then vary the angle
            for alpha in np.arange(0.0, 2.0*np.pi, 2.0*np.pi/10.0):
                direction_vec = np.array([np.cos(alpha), np.sin(alpha), 0])
                robot_pos[:2] = target_pos[:2] + r * direction_vec[:2]
                rotation = R.from_euler('z', np.pi + alpha, degrees=False)
                robot_orient = rotation.as_quat()

                # Put robot into this position
                self.move(robot_pos, robot_orient)
                if not self.check_collisions():
                    return True
        return False
