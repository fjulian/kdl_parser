import numpy as np
from scipy.spatial.transform import Rotation as R

class SceneTossing:
    def __init__(self, world):
        self._world = world
        self._world.add_plane()

        self.objects_list = []

        self.objects = [("duck_vhacd.urdf", 1.0), ("duck_vhacd.urdf", 1.0), ("duck_vhacd.urdf", 1.0),
                        ("block.urdf", 1.4), ("block.urdf", 1.4), ("block.urdf", 1.4),
                        ("block.urdf", 1.6), ("block.urdf", 1.6), ("block.urdf", 1.6),
                        ("block.urdf", 1.9), ("block.urdf", 1.9), ("block.urdf", 1.9),
                        ("block.urdf", 1.2), ("block.urdf", 1.2), ("block.urdf", 1.2),
                        ("block.urdf", 1.7), ("block.urdf", 1.7), ("block.urdf", 1.7)]
        self.objects_low = np.array([0.9, -0.3, 0.2])
        self.objects_high = np.array([1.1, 0.3, 1.0])

        self.add_objects()
        self.add_trays()

    def add_objects(self):
        for obj in self.objects:
            pos = np.random.uniform(self.objects_low, self.objects_high)
            yaw_angel_deg = np.random.uniform(0.0, 360.0, 1)[0]
            r1 = R.from_euler('z', yaw_angel_deg, degrees=True)
            orient = r1.as_quat()
            model = self._world.add_model(obj[0], pos, orient, scale=obj[1])
            self.objects_list.append(model)

    def add_trays(self):
        pos = [1.0, 0.0, 0.0]
        orient = [0.0, 0.0, 0.0, 1.0]
        scale = 1.2
        path = "tray/tray.urdf"
        self._world.add_model(path, pos, orient, scale=scale)

        # pos = [2.0, -0.5, 0.0]
        # scale = 0.8
        # self.add_model(path, pos, orient, scale=scale)
        #
        # pos = [2.0, 0.5, 0.0]
        # self.add_model(path, pos, orient, scale=scale)

    def reset():
        raise NotImplementedError
