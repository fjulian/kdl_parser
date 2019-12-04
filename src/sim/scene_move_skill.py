import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from tools.util import rotate_orient, ObjectInfo
from sim.cupboard import Cupboard

class SceneMoveSkill:
    def __init__(self, world, restored_objects=None):
        self._world = world

        if restored_objects is None:
            self._world.add_plane()

            self.objects = dict()
            # self.objects["table"] = ObjectInfo(urdf_path_=os.path.join(os.getcwd(),"data/models/table/table.urdf"),
            #                                 init_pos_=np.array([3.0, 0.0, 0.0]),
            #                                 init_orient_=np.array([0.0, 0.0, 0.0, 1.0])
            #                                 )
            # self.objects["cube1"] = ObjectInfo(urdf_path_="cube_small.urdf",
            #                                 init_pos_=np.array([2.5, 0.0, 0.7]),
            #                                 # init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
            #                                 init_orient_=rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), 'z', -20.0),
            #                                 grasp_pos_=[
            #                                     np.array([0.0, 0.0, 0.0]),
            #                                     np.array([0.0, 0.0, 0.0])
            #                                 ],
            #                                 grasp_orient_=[
            #                                     np.array([0.0, 0.0, 0.0, 1.0]),
            #                                     rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), 'y', -25.0)
            #                                 ]
            #                                 )
            # self.objects["container1"] = ObjectInfo(urdf_path_=os.path.join(os.getcwd(),"data/models/container/container_no_lid.urdf"),
            #                                 init_pos_=np.array([3.5, -0.25, 0.625]),
            #                                 init_orient_=np.array([0.0, 0.0, 0.0, 1.0])
            #                                 )
            # self.objects["lid1"] = ObjectInfo(urdf_path_=os.path.join(os.getcwd(),"data/models/container/lid.urdf"),
            #                                 init_pos_=np.array([3.5, -0.25, 0.625]),
            #                                 init_orient_=np.array([0.0, 0.0, 0.0, 1.0])
            #                                 )
            # self.objects["container2"] = ObjectInfo(urdf_path_=os.path.join(os.getcwd(),"data/models/container/container_sliding_lid.urdf"),
            #                                 init_pos_=np.array([3.5, 0.25, 0.625]),
            #                                 init_orient_=np.array([0.0, 0.0, 0.0, 1.0])
            #                                 )

            cupboard = Cupboard(world, pos_=[0.0, 2.0, 0.0], orient_=[0.0, 0.0, 0.0, 1.0])
            self.objects["cupboard"] = cupboard.get_info()

            self.add_objects()
        else:
            self.objects = restored_objects

    def add_objects(self):
        print("---------------------------")
        for key, obj in self.objects.items():
            if self.objects[key].model is None:
                self.objects[key].model = self._world.add_model(obj.urdf_path, obj.init_pos, obj.init_orient, scale=obj.scale)
            print("Added object "+key+". ID: "+str(obj.model.uid))
        print("---------------------------")

    def reset(self):
        raise NotImplementedError
