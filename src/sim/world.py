import pybullet as p
import os
import pybullet_data
import numpy as np
import time
from math import ceil
import atexit


class World:
    def __init__(self, gui_=True, sleep_=True):
        self.gui = gui_
        self.sleep_flag = sleep_
        if self.gui:
            # self.physics_client = p.connect(p.GUI)
            self.physics_client = p.connect(p.SHARED_MEMORY)
        else:
            self.physics_client = p.connect(p.DIRECT)
        p.resetSimulation(self.physics_client)

        p.setGravity(0, 0, -9.81, self.physics_client)

        self.sim_time = 0.0
        self.f_s = 240.0
        self.T_s = 1.0 / float(self.f_s)

        self.collision_checker = None
        self.cross_uid = ()

        self.forces = []

        self.velocity_setter = None

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        atexit.register(self.close)

    def sleep(self, seconds):
        if self.sleep_flag:
            time.sleep(seconds)

    def add_model(self, path, position, orientation, scale=1.0):
        model = _Model(self.physics_client)
        model.load(path, position, orientation, scale)
        return model

    def del_model(self, model):
        model.remove()

    def draw_cross(self, point):
        if len(self.cross_uid) > 0:
            p.removeUserDebugItem(self.cross_uid[0])
            p.removeUserDebugItem(self.cross_uid[1])
            p.removeUserDebugItem(self.cross_uid[2])
        start1 = point - np.array([0.1, 0.0, 0.0])
        end1 = point + np.array([0.1, 0.0, 0.0])
        start2 = point - np.array([0.0, 0.1, 0.0])
        end2 = point + np.array([0.0, 0.1, 0.0])
        start3 = point - np.array([0.0, 0.0, 0.1])
        end3 = point + np.array([0.0, 0.0, 0.1])
        color = np.array([255, 0, 0]) / 255.0
        width = 1.0
        lifetime = 0
        uid1 = p.addUserDebugLine(start1.tolist(), end1, color.tolist(), width, lifetime)
        uid2 = p.addUserDebugLine(start2.tolist(), end2, color.tolist(), width, lifetime)
        uid3 = p.addUserDebugLine(start3.tolist(), end3, color.tolist(), width, lifetime)
        self.cross_uid = (uid1, uid2, uid3)

    def step_one(self):
        for frc in self.forces:
            p.applyExternalForce(frc[0], frc[1], frc[2], frc[3], frc[4])
        if self.velocity_setter is not None:
            self.velocity_setter()
        p.stepSimulation()
        if self.collision_checker is not None:
            self.collision_checker()

    def step_seconds(self, secs):
        for _ in range(int(ceil(secs * self.f_s))):
            self.step_one()
            self.sleep(self.T_s)

    def add_plane(self):
        return p.loadURDF("plane.urdf")

    def close(self):
        print("Closing world")
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

    def remove(self):
        p.removeBody(self.uid)

    def get_pos(self):
        ret = p.getBasePositionAndOrientation(self.uid)
        pos = np.array(ret[0])
        orient = np.array(ret[1])
        return pos, orient
