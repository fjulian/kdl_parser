import os
import pickle
import pandas as pd
import atexit
import pybullet as p
import numpy as np


# To show dataframes:
# pd.options.display.max_columns=20
# pd.options.display.expand_frame_repr=False
# self.data["on"].iloc[:,:]


class PredicateDataManager:
    def __init__(self, basedir, scene):
        self.scene = scene
        self.data = dict()
        self.meta_data = dict()
        self.pred_dir = os.path.join(basedir, "data", "predicates")
        os.makedirs(self.pred_dir, exist_ok=True)
        atexit.register(self._save_pred_data)

    def extract_features(self, obj_name):
        obj_uid = self.scene.objects[obj_name].model.uid
        link2idx = self.scene.objects[obj_name].model.link_name_to_index
        aabb = np.array(p.getAABB(obj_uid))
        for link in link2idx:
            tmp = np.array(obj_uid, link2idx[link])
            aabb[0, :] = np.minimum(aabb[0, :], tmp[0, :])
            aabb[1, :] = np.minimum(aabb[1, :], tmp[1, :])

        com = np.mean(aabb, 0)
        return com, aabb

    def take_snapshot(self, name: str, raw_args: str, label: bool):
        data_exists = True
        if name not in self.data:
            data_exists = self._load_pred_data(name)
            if not data_exists:
                self.meta_data[name] = dict.fromkeys(["num_args"])

        arguments = raw_args.split(",")
        for i, a in enumerate(arguments):
            arguments[i] = a.strip()

        all_features = list()
        for arg in arguments:
            features = self.extract_features(arg)
            all_features.append(features[0])
            all_features.append(features[1][0, :])
            all_features.append(features[1][1, :])
        all_features = np.array(all_features).reshape((1, -1))

        if data_exists:
            assert self.meta_data[name]["num_args"] == len(arguments)
            current_length = self.data[name].shape[0]
            self.data[name].loc[current_length, :] = all_features
        else:
            self.meta_data[name]["num_args"] = len(arguments)
            cols = []
            for i in range(len(arguments)):
                cols.extend([f"a{i+1}_com_x", f"a{i+1}_com_y", f"a{i+1}_com_z"])
                cols.extend(
                    [f"a{i + 1}_bb_min_x", f"a{i + 1}_bb_min_y", f"a{i + 1}_bb_min_z"]
                )
                cols.extend(
                    [f"a{i + 1}_bb_max_x", f"a{i + 1}_bb_max_y", f"a{i + 1}_bb_max_z"]
                )
            self.data[name] = pd.DataFrame(all_features, columns=cols)

        return True

    def _get_file_name(self, name):
        return os.path.join(self.pred_dir, f"{name}.pkl")

    def _load_pred_data(self, name: str):
        assert name not in self.data
        filename = self._get_file_name(name)
        if os.path.isfile(filename):
            with open(filename, "rb") as f:
                content = pickle.load(f)
                self.meta_data[name] = content[0]
                self.data[name] = content[1]
            return True
        return False

    def _save_pred_data(self):
        for name in self.data:
            filename = self._get_file_name(name)
            content = (self.meta_data[name], self.data[name])
            with open(filename, "wb") as f:
                pickle.dump(content, f)
        print("Predicate data saved")
