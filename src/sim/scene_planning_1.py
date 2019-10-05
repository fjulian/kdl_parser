import numpy as np
from scipy.spatial.transform import Rotation as R
import os

class ScenePlanning1:
    def __init__(self, world):
        self._world = world
        self._world.add_plane()

        self.objects = []

        print(os.path.join(os.getcwd(),"data/table.urdf"))
        self.objects.append(ObjectInfo(name_ = "table",
                                        urdf_path_=os.path.join(os.getcwd(),"data/table/table.urdf"),
                                        init_pos_=[3.0, 0.0, 0.0],
                                        init_orient_=[0.0, 0.0, 0.0, 1.0]
                                        ))
        self.objects.append(ObjectInfo(name_ = "cube1",
                                        urdf_path_="cube_small.urdf",
                                        init_pos_=[2.5, 0.0, 1.5],
                                        init_orient_=[0.0, 0.0, 0.0, 1.0],
                                        
                                        ))

        self.add_objects()

    def add_objects(self):
        for obj in self.objects:
            obj.model = self._world.add_model(obj.urdf_path, obj.init_pos, obj.init_orient, scale=obj.scale)
            print("Added object "+obj.urdf_path+". ID: "+str(obj.model.uid))

    def reset(self):
        raise NotImplementedError


class ObjectInfo:
    def __init__(self, name_, urdf_path_, init_pos_, init_orient_, init_scale_=1.0, grasp_pos_=[], grasp_orient_=[]):
        self.name = name_
        self.urdf_path = urdf_path_
        self.init_pos = init_pos_
        self.init_orient = init_orient_
        self.scale = init_scale_
        self.grasp_pos = grasp_pos_
        self.grasp_orient = grasp_orient_
        self.model = None
