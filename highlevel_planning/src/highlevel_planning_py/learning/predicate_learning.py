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

    def capture_demonstration(self, name: str, arguments: list, label: bool):
        data_exists = True
        if name not in self.data:
            data_exists = self._load_pred_data(name)
            if not data_exists:
                self.meta_data[name] = dict.fromkeys(["num_args"])
                self.data[name] = pd.DataFrame()

        all_features = self.take_snapshot(arguments)

        self.data[name] = pd.concat(
            (self.data[name], all_features), axis=0, ignore_index=True
        )
        if data_exists:
            assert self.meta_data[name]["num_args"] == len(arguments)
            assert self.meta_data[name]["num_features"] == len(
                self.data[name].columns
            ) / len(arguments)
        else:
            self.meta_data[name]["num_args"] = len(arguments)
            self.meta_data[name]["num_features"] = len(self.data[name].columns) / len(
                arguments
            )

        return True

    def take_snapshot(self, arguments: list):
        all_features = list()
        for arg in arguments:
            features = self._extract_features(arg)
            all_features.append(features[0])
            all_features.append(features[1][0, :])
            all_features.append(features[1][1, :])
        all_features = np.array(all_features).reshape((1, -1))

        cols = []
        for i in range(len(arguments)):
            cols.extend([f"a{i + 1}_com_x", f"a{i + 1}_com_y", f"a{i + 1}_com_z"])
            cols.extend(
                [f"a{i + 1}_bb_min_x", f"a{i + 1}_bb_min_y", f"a{i + 1}_bb_min_z"]
            )
            cols.extend(
                [f"a{i + 1}_bb_max_x", f"a{i + 1}_bb_max_y", f"a{i + 1}_bb_max_z"]
            )

        all_features = pd.DataFrame(all_features, columns=cols)
        return all_features

    def _extract_features(self, obj_name):
        obj_uid = self.scene.objects[obj_name].model.uid
        link2idx = self.scene.objects[obj_name].model.link_name_to_index
        aabb = np.array(p.getAABB(obj_uid))
        for link in link2idx:
            tmp = np.array(obj_uid, link2idx[link])
            aabb[0, :] = np.minimum(aabb[0, :], tmp[0, :])
            aabb[1, :] = np.minimum(aabb[1, :], tmp[1, :])

        com = np.mean(aabb, 0)
        return com, aabb

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


class PredicateLearner:
    def __init__(
        self,
        data: pd.DataFrame,
        pdm: PredicateDataManager,
        meta_data=None,
        relative_arg: int = 0,
    ):
        self.data = data.copy(deep=True)
        self.meta_data = meta_data
        self.relative_arg = relative_arg
        self.pdm = pdm

        self.relative_arg_selector, self.non_relative_args_selector = (
            self.preprocess_data()
        )

        self.upper_rules_forall = None
        self.lower_rules_forall = None

    def preprocess_data(self):
        if self.meta_data["num_args"] < 2:
            return False
        non_relative_args = list(range(self.meta_data["num_args"]))
        non_relative_args.remove(self.relative_arg)
        non_relative_args_selector = list()
        for arg in non_relative_args:
            non_relative_args_selector.extend(
                np.array(range(self.meta_data["num_features"]))
                + arg * self.meta_data["num_features"]
            )
        relative_arg_selector = (
            np.array(range(self.meta_data["num_features"]))
            + self.relative_arg * self.meta_data["num_features"]
        )

        origin_arg = non_relative_args[0]
        origin_arg_selector = (
            np.array(range(self.meta_data["num_features"]))
            + origin_arg * self.meta_data["num_features"]
        )

        for i in range(3):
            self.data.iloc[:, i::3] = self.data.iloc[:, i::3].subtract(
                self.data.iloc[:, origin_arg_selector[i]], axis=0
            )

        return relative_arg_selector, non_relative_args_selector

    def build_rules(self):

        # Find upper and lower bounds
        upper_bounds = {i: pd.DataFrame() for i in range(3)}
        lower_bounds = {i: pd.DataFrame() for i in range(3)}
        for selector in self.relative_arg_selector:
            axis_idx = selector % 3
            this_non_relative_selector = self.non_relative_args_selector[axis_idx::3]

            # Check upper bounds
            this_upper_bounds = self.data.iloc[:, this_non_relative_selector].gt(
                self.data.iloc[:, selector], axis="index"
            )
            renamer = dict()
            for col in this_upper_bounds.columns:
                renamer[col] = f"{self.data.columns[selector]}<{col}"
            this_upper_bounds.rename(columns=renamer, inplace=True)
            upper_bounds[axis_idx] = pd.concat(
                [upper_bounds[axis_idx], this_upper_bounds], axis=1
            )

            # Check lower bounds
            this_lower_bounds = self.data.iloc[:, this_non_relative_selector].lt(
                self.data.iloc[:, selector], axis="index"
            )
            for col in renamer:
                renamer[col] = renamer[col].replace("<", ">")
            this_lower_bounds.rename(columns=renamer, inplace=True)
            lower_bounds[axis_idx] = pd.concat(
                [lower_bounds[axis_idx], this_lower_bounds], axis=1
            )

        # Extract candidates for rules that hold over all data
        self.upper_rules_forall = {i: upper_bounds[i].all(axis=0) for i in upper_bounds}
        self.lower_rules_forall = {i: lower_bounds[i].all(axis=0) for i in lower_bounds}

    def classify(self, arguments):
        pass

    def inquire(self):
        pass
