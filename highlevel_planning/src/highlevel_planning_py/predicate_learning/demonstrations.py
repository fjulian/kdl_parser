import os
import pickle
import pandas as pd
import atexit
import pybullet as p
import numpy as np
import rospy
from datetime import datetime


# To show dataframes:
# pd.options.display.max_columns=20
# pd.options.display.expand_frame_repr=False
# self.data["on"].iloc[:,:]


class PredicateDemonstrationManager:
    def __init__(self, basedir, scene):
        self.scene = scene
        self._data = dict()
        self._meta_data = dict()
        self.pred_dir = os.path.join(basedir, "data", "predicates")
        self.demo_dir = os.path.join(self.pred_dir, "demonstrations")
        os.makedirs(self.demo_dir, exist_ok=True)
        # atexit.register(self._save_pred_data)

    def capture_demonstration(self, name: str, arguments: list, label: bool):
        time_now = datetime.now()
        time_string = time_now.strftime("%y%m%d-%H%M%S")

        this_demo_dir = os.path.join(self.demo_dir, name, time_string)
        os.makedirs(this_demo_dir, exist_ok=False)

        # Save pickle
        save_data = (name, arguments, label, self.scene.objects)
        with open(os.path.join(this_demo_dir, "demo.pkl"), "wb") as output:
            pickle.dump(save_data, output)

        # Save human readable data
        save_string = f"Predicate name: {name}\nArguments: {arguments}\nHolds: {label}\nTime reported: {time_string}"
        with open(os.path.join(this_demo_dir, "info.txt"), "w") as output:
            pickle.dump(save_string, output)

        # Save bullet state
        p.saveBullet(os.path.join(this_demo_dir, "state.bullet"))

        # # Old ------------------
        # if not label:
        #     return False
        # data_exists = True
        # if name not in self._data:
        #     data_exists = self._load_pred_data(name)
        #     if not data_exists:
        #         self._meta_data[name] = dict.fromkeys(["num_args"])
        #         self._data[name] = pd.DataFrame()
        #
        # all_features = self.take_snapshot(arguments)
        #
        # self._data[name] = pd.concat(
        #     (self._data[name], all_features), axis=0, ignore_index=True
        # )
        # if data_exists:
        #     assert self._meta_data[name]["num_args"] == len(arguments)
        #     assert self._meta_data[name]["num_features"] == int(
        #         len(self._data[name].columns) / len(arguments)
        #     )
        # else:
        #     self._meta_data[name]["num_args"] = len(arguments)
        #     self._meta_data[name]["num_features"] = int(
        #         len(self._data[name].columns) / len(arguments)
        #     )

        return True

    # def take_snapshot(self, arguments: list):
    #     all_features = list()
    #     for arg in arguments:
    #         features = self._extract_features(arg)
    #         all_features.append(features[0])
    #         all_features.append(features[1][0, :])
    #         all_features.append(features[1][1, :])
    #     all_features = np.array(all_features).reshape((1, -1))
    #
    #     cols = []
    #     for i in range(len(arguments)):
    #         cols.extend([f"a{i + 1}_com_x", f"a{i + 1}_com_y", f"a{i + 1}_com_z"])
    #         cols.extend(
    #             [f"a{i + 1}_bb_min_x", f"a{i + 1}_bb_min_y", f"a{i + 1}_bb_min_z"]
    #         )
    #         cols.extend(
    #             [f"a{i + 1}_bb_max_x", f"a{i + 1}_bb_max_y", f"a{i + 1}_bb_max_z"]
    #         )
    #
    #     all_features = pd.DataFrame(all_features, columns=cols)
    #     return all_features

    # def get_meta_data(self, pred_name):
    #     if not self._check_exists(pred_name):
    #         raise ValueError
    #     return self._meta_data[pred_name]

    # def get_data(self, pred_name):
    #     if not self._check_exists(pred_name):
    #         raise ValueError
    #     return self._data[pred_name]

    # def set_object_position(self, object_name, desired_com_pos):
    #     object_uid = self.scene.objects[object_name].model.uid
    #     pos, orient = p.getBasePositionAndOrientation(object_uid)
    #     pos, orient = np.array(pos), np.array(orient)
    #     com, aabb = self._extract_features(object_name)
    #     diff_base_com = pos - com
    #
    #     desired_base_pos = desired_com_pos + diff_base_com
    #
    #     p.resetBasePositionAndOrientation(object_uid, desired_base_pos, orient)

    # def _check_exists(self, pred_name):
    #     if pred_name not in self._data:
    #         exists = self._load_pred_data(pred_name)
    #         if not exists:
    #             return False
    #     return True

    # def _extract_features(self, obj_name):
    #     obj_uid = self.scene.objects[obj_name].model.uid
    #     link2idx = self.scene.objects[obj_name].model.link_name_to_index
    #     aabb = np.array(p.getAABB(obj_uid))
    #     for link in link2idx:
    #         tmp = np.array(p.getAABB(obj_uid, link2idx[link]))
    #         aabb[0, :] = np.minimum(aabb[0, :], tmp[0, :])
    #         aabb[1, :] = np.maximum(aabb[1, :], tmp[1, :])
    #
    #     com = np.mean(aabb, 0)
    #     return com, aabb

    # def _get_file_name(self, name):
    #     return os.path.join(self.pred_dir, f"{name}.pkl")

    # def _load_pred_data(self, name: str):
    #     assert name not in self._data
    #     filename = self._get_file_name(name)
    #     if os.path.isfile(filename):
    #         with open(filename, "rb") as f:
    #             content = pickle.load(f)
    #             self._meta_data[name] = content[0]
    #             self._data[name] = content[1]
    #         return True
    #     return False

    # def _save_pred_data(self):
    #     for name in self._data:
    #         filename = self._get_file_name(name)
    #         content = (self._meta_data[name], self._data[name])
    #         with open(filename, "wb") as f:
    #             pickle.dump(content, f)
    #     print("Predicate data saved")
