import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import py_trees


class ActionNavigate(py_trees.behaviour.Behaviour):
    def __init__(self, scene, robot_uid, name="nav_action"):
        super(ActionNavigate, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))


def nav_process(pipe_connection, scene, robot_uid):
    pass



def _check_collisions(scene, robot_uid):
    for _, obj in scene.objects.items():
        temp = p.getClosestPoints(robot_uid, obj.model.uid, distance=0.5)
        for elem in temp:
            contact_distance = elem[8]
            if contact_distance < 0.0:
                # print("There is a collision")
                return True
    return False

def _move(robot_uid, pos, orient):
    p.resetBasePositionAndOrientation(robot_uid, pos.tolist(), orient.tolist())

def move_to_object(target_name, scene, robot_uid):
    target_id = scene.objects[target_name].model.uid

    # Get the object position
    temp = p.getBasePositionAndOrientation(target_id)
    target_pos = np.array(temp[0])

    # Get robot position
    temp = p.getBasePositionAndOrientation(robot_uid)
    robot_pos = np.array(temp[0])

    # Get valid nav angles
    nav_angle = scene.objects[target_name].nav_angle
    if nav_angle is None:
        alphas = np.arange(0.0, 2.0*np.pi, 2.0*np.pi/10.0)
    else:
        alphas = np.array([nav_angle])
    nav_min_dist = scene.objects[target_name].nav_min_dist
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
            rotation = R.from_euler('z', np.pi + alpha, degrees=False)
            robot_orient = rotation.as_quat()

            # Put robot into this position
            _move(robot_uid, robot_pos, robot_orient)
            if not _check_collisions(scene, robot_uid):
                return True
    return False
