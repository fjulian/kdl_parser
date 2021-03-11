import numpy as np
import os
from highlevel_planning_py.tools.util import rotate_orient, ObjectInfo
from highlevel_planning_py.sim.scene_base import SceneBase
from highlevel_planning_py.sim.cupboard import get_cupboard_info


class ScenePlanning1(SceneBase):
    def __init__(self, world, base_dir, restored_objects=None):
        SceneBase.__init__(self, world, base_dir, restored_objects)

        if restored_objects is None:
            self.objects["table"] = ObjectInfo(
                urdf_name_="table/table.urdf",
                urdf_path_=os.path.join(base_dir, "table/table.urdf"),
                init_pos_=np.array([3.0, 0.0, 0.0]),
                init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
            )
            self.objects["cube1"] = ObjectInfo(
                urdf_path_="cube_small.urdf",
                init_pos_=np.array([2.5, 0.0, 0.7]),
                # init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
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
            self.objects["lego"] = ObjectInfo(
                urdf_path_="lego/lego.urdf",
                init_pos_=np.array([2.45, -0.3, 0.68]),
                init_orient_=rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), "z", 0.0),
                init_scale_=2.0,
                grasp_links_=[-1],
                grasp_pos_={-1: [np.array([0.0, 0.0, 0.0])]},
                grasp_orient_={-1: [np.array([0.0, 0.0, 0.0, 1.0])]},
            )
            self.objects["duck"] = ObjectInfo(
                urdf_path_="duck_vhacd.urdf",
                init_pos_=np.array([2.8, -0.25, 0.67]),
                init_orient_=rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), "x", 90.0),
                init_scale_=0.8,
                grasp_links_=[-1],
                grasp_pos_={-1: [np.array([-0.015, 0.03, 0.0])]},
                grasp_orient_={
                    -1: [rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), "x", -90.0)]
                },
            )
            self.objects["container1"] = ObjectInfo(
                urdf_name_="container/container_no_lid.urdf",
                urdf_path_=os.path.join(base_dir, "container/container_no_lid.urdf"),
                init_pos_=np.array([3.5, -0.25, 0.625]),
                init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
            )
            self.objects["lid1"] = ObjectInfo(
                urdf_name_="container/lid.urdf",
                urdf_path_=os.path.join(base_dir, "container/lid.urdf"),
                init_pos_=np.array([3.5, -0.25, 0.775]),
                init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
                grasp_pos_={5: [np.array([0.0, 0.0, 0.0])]},
                grasp_orient_={
                    5: [rotate_orient(np.array([0.0, 0.0, 0.0, 1.0]), "y", 90)]
                },
                grasp_links_=[5],
                friction_setting_=[{"link_name": "handle", "lateral_friction": 1.0}],
            )
            self.objects["container2"] = ObjectInfo(
                urdf_name_="container/container_sliding_lid.urdf",
                urdf_path_=os.path.join(
                    base_dir, "container/container_sliding_lid.urdf"
                ),
                init_pos_=np.array([3.5, 0.25, 0.625]),
                init_orient_=np.array([0.0, 0.0, 0.0, 1.0]),
            )
            self.objects["cupboard"] = get_cupboard_info(
                base_dir, pos=[0.0, 2.0, 0.0], orient=[0.0, 0.0, 0.0, 1.0]
            )

            self.add_objects()
