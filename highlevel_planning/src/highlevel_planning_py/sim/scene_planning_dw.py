import numpy as np
import os
from highlevel_planning_py.tools.util import rotate_orient, ObjectInfo
from highlevel_planning_py.sim.scene_base import SceneBase
from highlevel_planning_py.sim.cupboard import get_cupboard_info

from dishwasher_challenge.sim import _find_objects


class ScenePlanningDW(SceneBase):
    def __init__(self, world, assets_dir, restored_objects=None):
        SceneBase.__init__(self, world, assets_dir, restored_objects)

        if restored_objects is None:
            self.objects["kitchen"] = ObjectInfo(
                urdf_path_="kitchen.urdf",
                init_pos_=np.array([0.0, 0.0, 0.0]),
                init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
            )

            available_objects = _find_objects(assets_dir)
            selected_objects = list()
            for obj in available_objects:
                obj_class = obj.split("/")[-2]
                if obj_class != "plate":
                    continue
                obj_num = int(obj.split("/")[-1].split(".")[-2])
                if obj_num != 3:
                    continue
                selected_objects.append(obj)
                break
            self.objects["plate"] = ObjectInfo(
                urdf_path_=selected_objects[0],
                init_pos_=np.array([0.0, -1.0, 0.1]),
                init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
            )

            self.add_objects()
