import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from tools.util import rotate_orient

class ScenePlanning1:
    def __init__(self, world):
        self._world = world
        self._world.add_plane()

        self.objects = dict()
        self.objects["table"] = ObjectInfo(urdf_path_=os.path.join(os.getcwd(),"data/table/table.urdf"),
                                        init_pos_=np.array([3.0, 0.0, 0.0]),
                                        init_orient_=np.array([0.0, 0.0, 0.0, 1.0])
                                        )
        self.objects["cube1"] = ObjectInfo(urdf_path_="cube_small.urdf",
                                        init_pos_=np.array([2.5, 0.0, 0.7]),
                                        # init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
                                        init_orient_=rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), 'z', -20.0),
                                        grasp_pos_=[
                                            np.array([0.0, 0.0, 0.0]),
                                            np.array([0.0, 0.0, 0.0])
                                        ],
                                        grasp_orient_=[
                                            np.array([0.0, 0.0, 0.0, 1.0]),
                                            rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), 'y', -25.0)
                                        ]
                                        )

        self.add_objects()

    def add_objects(self):
        for key, obj in self.objects.items():
            self.objects[key].model = self._world.add_model(obj.urdf_path, obj.init_pos, obj.init_orient, scale=obj.scale)
            print("Added object "+obj.urdf_path+". ID: "+str(obj.model.uid))

    def reset(self):
        raise NotImplementedError


class ObjectInfo:
    def __init__(self, urdf_path_, init_pos_, init_orient_, init_scale_=1.0, grasp_pos_=[], grasp_orient_=[]):
        self.urdf_path = urdf_path_
        self.init_pos = init_pos_
        self.init_orient = init_orient_
        self.scale = init_scale_
        self.grasp_pos = grasp_pos_
        self.grasp_orient = grasp_orient_
        self.model = None
