import pybullet as p
import os
import pybullet_data
import numpy as np
import time
from math import floor
from scipy.spatial.transform import Rotation as R


class World:
    def __init__(self, gui_=True, sleep_=True):
        self.gui = gui_
        self.sleep_flag = sleep_
        if self.gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setGravity(0, 0, -9.81, self.physics_client)

        self.sim_time = 0.0
        self.f_s = 240.0
        self.T_s = 1.0 / float(self.f_s)

        self.models = []
        self.objects_list = []
        self.plane_id = 0

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.objects = [("duck_vhacd.urdf", 1.0), ("duck_vhacd.urdf", 1.0), ("duck_vhacd.urdf", 1.0),
                        ("block.urdf", 1.4), ("block.urdf", 1.4), ("block.urdf", 1.4),
                        ("block.urdf", 1.6), ("block.urdf", 1.6), ("block.urdf", 1.6),
                        ("block.urdf", 1.9), ("block.urdf", 1.9), ("block.urdf", 1.9),
                        ("block.urdf", 1.2), ("block.urdf", 1.2), ("block.urdf", 1.2),
                        ("block.urdf", 1.7), ("block.urdf", 1.7), ("block.urdf", 1.7)]
        self.objects_low = np.array([0.3, -0.3, 0.2])
        self.objects_high = np.array([0.6, 0.3, 1.0])
        
        self.collision_checker = None

        self.cross_uid = ()

    def sleep(self, seconds):
        if self.sleep_flag:
            time.sleep(seconds)

    def add_model(self, path, position, orientation, scale=1.0):
        model = _Model(self.physics_client)
        model.load(path, position, orientation, scale)
        self.models.append(model)
        return model

    def del_model(self, model):
        model.remove()
        self. models.remove(model)

    def draw_cross(self, point):
        if len(self.cross_uid) > 0:
            p.removeUserDebugItem(self.cross_uid[0])
            p.removeUserDebugItem(self.cross_uid[1])
        start1 = point - np.array([0.1, 0.0, 0.0])
        end1 = point + np.array([0.1, 0.0, 0.0])
        start2 = point - np.array([0.0, 0.1, 0.0])
        end2 = point + np.array([0.0, 0.1, 0.0])
        color = np.array([255, 0, 0]) / 255.0
        width = 1.0
        lifetime = 0
        uid1 = p.addUserDebugLine(start1.tolist(), end1, color.tolist(), width, lifetime)
        uid2 = p.addUserDebugLine(start2.tolist(), end2, color.tolist(), width, lifetime)
        self.cross_uid = (uid1, uid2)

    def step_one(self):
        p.stepSimulation()
        if self.collision_checker is not None:
            self.collision_checker()

    def step_seconds(self, secs):
        for i in range(int(floor(secs * 240))):
            self.step_one()
            self.sleep(self.T_s)

    def add_objects(self):
        for obj in self.objects:
            pos = np.random.uniform(self.objects_low, self.objects_high)
            yaw_angel_deg = np.random.uniform(0.0, 360.0, 1)[0]
            r1 = R.from_euler('z', yaw_angel_deg, degrees=True)
            orient = r1.as_quat()
            model = self.add_model(obj[0], pos, orient, scale=obj[1])
            self.objects_list.append(model)

    def add_block(self):
        pos = [0.5, 0.2, 0.0]
        orient = [0.0, 0.0, 0.0, 1.0]
        model = self.add_model("block.urdf", pos, orient, scale=2)
        self.objects_list.append(model)

    def add_trays(self):
        pos = [0.46, 0.0, 0.0]
        orient = [0.0, 0.0, 0.0, 1.0]
        scale = 1.2
        path = "tray/tray.urdf"
        self.add_model(path, pos, orient, scale=scale)

        # pos = [2.0, -0.5, 0.0]
        # scale = 0.8
        # self.add_model(path, pos, orient, scale=scale)
        #
        # pos = [2.0, 0.5, 0.0]
        # self.add_model(path, pos, orient, scale=scale)

    def add_plane(self):
        self.plane_id = p.loadURDF("plane.urdf")

    def close(self):
        p.disconnect(self.physics_client)


class _Model:
    def __init__(self, physics_client):
        self._physics_client = physics_client
        self.uid = 0
        self.name = ''

    def load(self, path, position, orientation, scale):
        model_path = os.path.expanduser(path)
        self.uid = p.loadURDF(model_path, position, orientation, globalScaling=scale,
                              physicsClientId=self._physics_client)
        self.name = p.getBodyInfo(self.uid)
        # print(f"{self.name}")

    def remove(self):
        p.removeBody(self.uid)

    def get_pos(self):
        ret = p.getBasePositionAndOrientation(self.uid)
        pos = np.array(ret[0])
        orient = np.array(ret[1])
        return pos, orient
