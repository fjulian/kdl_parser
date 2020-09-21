import numpy as np
import os
from highlevel_planning.sim.scene_base import SceneBase
from highlevel_planning.tools.util import rotate_orient, ObjectInfo


class SceneTest1(SceneBase):
    def __init__(self, world, base_dir, restored_objects=None):
        SceneBase.__init__(self, world, base_dir, restored_objects)

        if restored_objects is None:
            self._world.add_plane()

            self.objects = dict()
            self.objects["table"] = ObjectInfo(
                urdf_path_=os.path.join(base_dir, "data/models/table/table.urdf"),
                init_pos_=np.array([3.0, 0.0, 0.0]),
                init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
            )
            self.objects["cube1"] = ObjectInfo(
                urdf_path_="cube_small.urdf",
                init_pos_=np.array([2.5, 0.0, 0.7]),
                init_orient_=rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), "z", -20.0),
                grasp_links_=[-1],
                grasp_pos_={-1: [np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])]},
                grasp_orient_={
                    -1: [
                        np.array([0.0, 0.0, 0.0, 1.0]),
                        rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), "y", -25.0),
                    ]
                },
            )
            self.objects["container1"] = ObjectInfo(
                urdf_path_=os.path.join(
                    base_dir, "data/models/container/container_no_lid.urdf"
                ),
                init_pos_=np.array([3.5, -0.25, 0.625]),
                init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
            )
            self.objects["cube2"] = ObjectInfo(
                urdf_path_="cube_small.urdf",
                init_pos_=np.array([3.5, -0.25, 0.77]),
                init_orient_=rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), "z", -20.0),
                grasp_links_=[-1],
                grasp_pos_={-1: [np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])]},
                grasp_orient_={
                    -1: [
                        np.array([0.0, 0.0, 0.0, 1.0]),
                        rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), "y", -25.0),
                    ]
                },
            )
            self.add_objects()
