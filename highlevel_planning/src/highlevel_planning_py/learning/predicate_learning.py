import os
import pickle
import pandas as pd
import atexit
import pybullet as p
import numpy as np
import rospy

# To show dataframes:
# pd.options.display.max_columns=20
# pd.options.display.expand_frame_repr=False
# self.data["on"].iloc[:,:]


class PredicateDataManager:
    def __init__(self, basedir, scene):
        self.scene = scene
        self._data = dict()
        self._meta_data = dict()
        self.pred_dir = os.path.join(basedir, "data", "predicates")
        os.makedirs(self.pred_dir, exist_ok=True)
        atexit.register(self._save_pred_data)

    def capture_demonstration(self, name: str, arguments: list, label: bool):
        data_exists = True
        if name not in self._data:
            data_exists = self._load_pred_data(name)
            if not data_exists:
                self._meta_data[name] = dict.fromkeys(["num_args"])
                self._data[name] = pd.DataFrame()

        all_features = self.take_snapshot(arguments)

        self._data[name] = pd.concat(
            (self._data[name], all_features), axis=0, ignore_index=True
        )
        if data_exists:
            assert self._meta_data[name]["num_args"] == len(arguments)
            assert self._meta_data[name]["num_features"] == len(
                self._data[name].columns
            ) / len(arguments)
        else:
            self._meta_data[name]["num_args"] = len(arguments)
            self._meta_data[name]["num_features"] = len(self._data[name].columns) / len(
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

    def get_meta_data(self, pred_name):
        if not self._check_exists(pred_name):
            raise ValueError
        return self._meta_data[pred_name]

    def get_data(self, pred_name):
        if not self._check_exists(pred_name):
            raise ValueError
        return self._data[pred_name]

    def _check_exists(self, pred_name):
        if pred_name not in self._data:
            exists = self._load_pred_data(pred_name)
            if not exists:
                return False
        return True

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
        assert name not in self._data
        filename = self._get_file_name(name)
        if os.path.isfile(filename):
            with open(filename, "rb") as f:
                content = pickle.load(f)
                self._meta_data[name] = content[0]
                self._data[name] = content[1]
            return True
        return False

    def _save_pred_data(self):
        for name in self._data:
            filename = self._get_file_name(name)
            content = (self._meta_data[name], self._data[name])
            with open(filename, "wb") as f:
                pickle.dump(content, f)
        print("Predicate data saved")


class RuleData:
    def __init__(self):
        self.processed_data = None
        self.meta_data = None
        self.relative_arg = None
        self.relative_arg_selector = None
        self.non_relative_args_selector = None
        self.non_relative_args = None
        self.origin_arg = None
        self.upper_rules_forall = None
        self.lower_rules_forall = None


class PredicateLearner:
    def __init__(self, pdm: PredicateDataManager):
        self.pdm = pdm
        self.data = dict()

    def build_rules(self, pred_name: str, relative_arg: int = 0):
        self._prepare_data(pred_name, relative_arg)
        rule_data = self.data[pred_name]

        # Find upper and lower bounds
        gt_relative_arg, lt_relative_arg = self._compare_values(
            rule_data.processed_data, rule_data
        )

        # Extract candidates for rules that hold over all data
        rule_data.upper_rules_forall = {
            i: gt_relative_arg[i].all(axis=0) for i in gt_relative_arg
        }
        rule_data.lower_rules_forall = {
            i: lt_relative_arg[i].all(axis=0) for i in lt_relative_arg
        }

    def classify(self, pred_name: str, arguments: list, relative_arg: int = 0):
        self._prepare_data(pred_name, relative_arg)
        rule_data = self.data[pred_name]

        sample = self.pdm.take_snapshot(arguments)
        processed_sample = self.preprocess_data(sample, rule_data)

        gt_relative_arg, lt_relative_arg = self._compare_values(
            processed_sample, rule_data
        )
        for i in range(3):
            assert (
                gt_relative_arg[i].columns == rule_data.upper_rules_forall[i].index
            ).all()
            assert (
                lt_relative_arg[i].columns == rule_data.lower_rules_forall[i].index
            ).all()
            assert gt_relative_arg[i].shape[0] == 1 and gt_relative_arg[i].shape[0] == 1
            assert lt_relative_arg[i].shape[0] == 1 and lt_relative_arg[i].shape[0] == 1

        result = True
        for i in range(3):
            result &= (
                gt_relative_arg[i]
                .iloc[0, :][rule_data.upper_rules_forall[i] == True]
                .all()
            )
        rospy.loginfo(f"Classification result: {result}")
        return result

    def inquire(self, pred_name: str, relative_arg: int = 0):
        self._prepare_data(pred_name, relative_arg)
        return True

    def _prepare_data(self, pred_name: str, relative_arg: int):
        if pred_name not in self.data:
            new_rule_data = RuleData()
            new_rule_data.meta_data = self.pdm.get_meta_data(pred_name)
            new_rule_data.relative_arg = relative_arg

            self.setup_selectors(new_rule_data)

            self.data[pred_name] = new_rule_data

        self.data[pred_name].processed_data = self.preprocess_data(
            self.pdm.get_data(pred_name), self.data[pred_name]
        )

    @staticmethod
    def setup_selectors(rule_data: RuleData):
        if rule_data.meta_data["num_args"] < 2:
            return False
        non_relative_args = list(range(rule_data.meta_data["num_args"]))
        non_relative_args.remove(rule_data.relative_arg)
        non_relative_args_selector = list()
        for arg in non_relative_args:
            non_relative_args_selector.extend(
                np.array(range(rule_data.meta_data["num_features"]))
                + arg * rule_data.meta_data["num_features"]
            )
        relative_arg_selector = (
            np.array(range(rule_data.meta_data["num_features"]))
            + rule_data.relative_arg * rule_data.meta_data["num_features"]
        )

        origin_arg = non_relative_args[0]

        rule_data.relative_arg_selector = relative_arg_selector
        rule_data.non_relative_args_selector = non_relative_args_selector
        rule_data.non_relative_args = non_relative_args
        rule_data.origin_arg = origin_arg

    @staticmethod
    def preprocess_data(raw_data: pd.DataFrame, rule_data: RuleData):
        origin_arg_selector = (
            np.array(range(rule_data.meta_data["num_features"]))
            + rule_data.origin_arg * rule_data.meta_data["num_features"]
        )

        processed_data = raw_data.copy(deep=True)
        for i in range(3):
            processed_data.iloc[:, i::3] = processed_data.iloc[:, i::3].subtract(
                processed_data.iloc[:, origin_arg_selector[i]], axis=0
            )
        return processed_data

    @staticmethod
    def _compare_values(data, rule_data):
        # Find upper and lower bounds
        gt_relative_arg = {i: pd.DataFrame() for i in range(3)}
        lt_relative_arg = {i: pd.DataFrame() for i in range(3)}
        for selector in rule_data.relative_arg_selector:
            axis_idx = selector % 3
            this_non_relative_selector = rule_data.non_relative_args_selector[
                axis_idx::3
            ]

            # Check upper bounds
            this_gt_relative_arg = data.iloc[:, this_non_relative_selector].gt(
                data.iloc[:, selector], axis="index"
            )
            renamer = dict()
            for col in this_gt_relative_arg.columns:
                renamer[col] = f"{data.columns[selector]}<{col}"
            this_gt_relative_arg.rename(columns=renamer, inplace=True)
            gt_relative_arg[axis_idx] = pd.concat(
                [gt_relative_arg[axis_idx], this_gt_relative_arg], axis=1
            )

            # Check lower bounds
            this_lt_relative_arg = data.iloc[:, this_non_relative_selector].lt(
                data.iloc[:, selector], axis="index"
            )
            for col in renamer:
                renamer[col] = renamer[col].replace("<", ">")
            this_lt_relative_arg.rename(columns=renamer, inplace=True)
            lt_relative_arg[axis_idx] = pd.concat(
                [lt_relative_arg[axis_idx], this_lt_relative_arg], axis=1
            )
        return gt_relative_arg, lt_relative_arg
