import os
import atexit
import pickle
import numpy as np
import rospy


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
        self.rules_inquired = (
            dict()
        )  # if the value for a rule is true, the rule was confirmed


class RuleDataManager:
    def __init__(self, basedir, feature_manager):
        self.data = dict()

        pred_dir = os.path.join(basedir, "data", "predicates")
        self.feature_dir = os.path.join(pred_dir, "features")
        self.rule_dir = os.path.join(pred_dir, "rules")
        os.makedirs(self.rule_dir, exist_ok=True)

        self.pfm = feature_manager

        self.open_inquiry = None

    def build_rules(self, pred_name: str):
        # Load features
        feature_file = os.path.join(self.feature_dir, f"{pred_name}.pkl")
        with open(feature_file, "rb") as f:
            feature_data, feature_meta_data = pickle.load(f)

        num_entities = len(feature_data)
        num_demos = len(feature_meta_data["demos_processed"])
        num_demo_counts = feature_data["arg0"].label.value_counts()
        num_positive_demos = num_demo_counts[True]
        skip_cols = 1
        num_values = np.zeros((num_entities,), dtype=int)
        num_features = np.zeros((num_entities,), dtype=int)
        for arg_idx in range(num_entities):
            num_values[arg_idx] = len(feature_data[f"arg{arg_idx}"].columns) - skip_cols
            assert num_values[arg_idx] % 3 == 0
            num_features[arg_idx] = num_values[arg_idx] / 3

        positive_indices = np.where(feature_data["arg0"].label)[0]

        # Make value comparisons
        total_num_values = int(np.sum(num_values))
        comparisons_positive = np.empty(
            (total_num_values, total_num_values, num_positive_demos)
        )
        comparisons_positive[:] = np.nan
        for first_entity_idx in range(num_entities - 1):
            for first_feature_idx in range(num_features[first_entity_idx]):
                for second_entity_idx in range(first_entity_idx + 1, num_entities):
                    for second_feature_idx in range(num_features[second_entity_idx]):
                        for i in range(3):
                            row_idx = int(
                                np.sum(num_values[:first_entity_idx])
                                + first_feature_idx * 3
                                + i
                            )
                            col_idx = int(
                                np.sum(num_values[:second_entity_idx])
                                + second_feature_idx * 3
                                + i
                            )
                            comparisons_positive[row_idx, col_idx, :] = (
                                feature_data[f"arg{first_entity_idx}"].iloc[
                                    positive_indices,
                                    skip_cols + first_feature_idx * 3 + i,
                                ]
                                > feature_data[f"arg{second_entity_idx}"].iloc[
                                    positive_indices,
                                    skip_cols + second_feature_idx * 3 + i,
                                ]
                            )

        accumulated = np.sum(comparisons_positive, axis=2)
        # TODO Figure out how to deal with nan

        # Extract rule candidates
        rule_candidates = list()
        lower = np.where(accumulated == 0)
        for candidate_idx in range(len(lower[0])):
            first_idx = lower[0][candidate_idx]
            second_idx = lower[1][candidate_idx]
            rule_candidates.append(
                self._construct_candidate(
                    second_idx, first_idx, num_values, skip_cols, feature_data
                )
            )
        higher = np.where(accumulated == comparisons_positive.shape[2])
        for candidate_idx in range(len(higher[0])):
            first_idx = higher[0][candidate_idx]
            second_idx = higher[1][candidate_idx]
            rule_candidates.append(
                self._construct_candidate(
                    first_idx, second_idx, num_values, skip_cols, feature_data
                )
            )

        rule_file = os.path.join(self.rule_dir, f"{pred_name}.pkl")
        with open(rule_file, "wb") as f:
            pickle.dump(rule_candidates, f)
        return True

    def classify(self, pred_name, arguments):
        rule_file = os.path.join(self.rule_dir, f"{pred_name}.pkl")
        with open(rule_file, "rb") as f:
            rule_candidates = pickle.load(f)

        features = self.pfm.extract_outer_features(arguments)

        res = True
        for rule in rule_candidates:
            larger_value = features[f"arg{rule['larger_idx'][0]}"].iloc[
                0, rule["larger_idx"][1]
            ]
            smaller_value = features[f"arg{rule['smaller_idx'][0]}"].iloc[
                0, rule["smaller_idx"][1]
            ]
            this_res = larger_value > smaller_value
            res &= this_res
        rospy.loginfo(f"Classification result: {res}")
        return res

    def _construct_candidate(
        self, larger_idx, smaller_idx, num_values, skip_cols, feature_data
    ):
        candidate = dict()
        larger_arg_idx, larger_feature_idx = self._compute_arg_idx(
            larger_idx, num_values
        )
        smaller_arg_idx, smaller_feature_idx = self._compute_arg_idx(
            smaller_idx, num_values
        )
        candidate["larger_idx"] = (larger_arg_idx, larger_feature_idx)
        candidate["smaller_idx"] = (smaller_arg_idx, smaller_feature_idx)
        candidate["larger_name"] = feature_data[f"arg{larger_arg_idx}"].columns[
            larger_feature_idx + skip_cols
        ]
        candidate["smaller_name"] = feature_data[f"arg{smaller_arg_idx}"].columns[
            smaller_feature_idx + skip_cols
        ]
        return candidate

    def _compute_arg_idx(self, full_idx, num_values):
        arg_idx = None
        for i in range(len(num_values)):
            if np.sum(num_values[: i + 1]) > full_idx:
                arg_idx = i
                break
        feature_idx = full_idx - np.sum(num_values[:arg_idx])
        return arg_idx, feature_idx

    # def build_rules(self, pred_name: str, relative_arg: int = 0):
    #     self._prepare_data(pred_name, relative_arg)
    #     rule_data = self.data[pred_name]
    #
    #     # Find upper and lower bounds
    #     gt_relative_arg, lt_relative_arg = self._compare_values(
    #         rule_data.processed_data, rule_data
    #     )
    #
    #     # Extract candidates for rules that hold over all data
    #     rule_data.upper_rules_forall = {
    #         i: gt_relative_arg[i].all(axis=0) for i in gt_relative_arg
    #     }
    #     rule_data.lower_rules_forall = {
    #         i: lt_relative_arg[i].all(axis=0) for i in lt_relative_arg
    #     }
    #
    #     # Set rules that were previously demonstrated through inquiries
    #     for confirmed_rule in rule_data.rules_inquired:
    #         if rule_data.rules_inquired[confirmed_rule]:
    #             assert self._get_value_given_rule(rule_data, confirmed_rule)
    #         else:
    #             self._set_value_given_rule(rule_data, confirmed_rule, False)
    #
    #     # Count and summarize rules
    #     rule_labels = list()
    #     for i in range(3):
    #         rule_labels.extend(
    #             rule_data.upper_rules_forall[i].index[
    #                 rule_data.upper_rules_forall[i] == True
    #             ]
    #         )
    #         rule_labels.extend(
    #             rule_data.lower_rules_forall[i].index[
    #                 rule_data.lower_rules_forall[i] == True
    #             ]
    #         )
    #     rospy.loginfo(f"Rules found ({len(rule_labels)}):")
    #     rospy.loginfo(rule_labels)

    # def classify(self, pred_name: str, arguments: list, relative_arg: int = 0):
    #     self._prepare_data(pred_name, relative_arg)
    #     rule_data = self.data[pred_name]
    #
    #     if rule_data.upper_rules_forall is None or rule_data.lower_rules_forall is None:
    #         self.build_rules(pred_name, relative_arg)
    #
    #     sample = self.pdm.take_snapshot(arguments)
    #     processed_sample = self.preprocess_data(sample, rule_data)
    #
    #     gt_relative_arg, lt_relative_arg = self._compare_values(
    #         processed_sample, rule_data
    #     )
    #
    #     # Check that data is compatible with sample
    #     for i in range(3):
    #         assert gt_relative_arg[i].shape[0] == 1 and lt_relative_arg[i].shape[0] == 1
    #         gt_relative_arg[i] = gt_relative_arg[i].iloc[0, :]
    #         lt_relative_arg[i] = lt_relative_arg[i].iloc[0, :]
    #
    #         assert np.all(
    #             gt_relative_arg[i].index == rule_data.upper_rules_forall[i].index
    #         )
    #         assert np.all(
    #             lt_relative_arg[i].index == rule_data.lower_rules_forall[i].index
    #         )
    #
    #     # Check that rules match, record any rules that are broken
    #     result = True
    #     broken_rules = list()
    #     for i in range(3):
    #         upper_rules = rule_data.upper_rules_forall[i]
    #         result &= gt_relative_arg[i][upper_rules == True].all()
    #         broken_rules.extend(
    #             upper_rules.index[
    #                 gt_relative_arg[i].ne(upper_rules)
    #                 & upper_rules.where(upper_rules == True)
    #             ]
    #         )
    #         lower_rules = rule_data.lower_rules_forall[i]
    #         result &= lt_relative_arg[i][lower_rules == True].all()
    #         broken_rules.extend(
    #             lower_rules.index[
    #                 lt_relative_arg[i].ne(lower_rules)
    #                 & lower_rules.where(lower_rules == True)
    #             ]
    #         )
    #     rospy.loginfo(f"Classification result: {result}")
    #     if not result:
    #         rospy.loginfo("Broken rules:")
    #         rospy.loginfo(broken_rules)
    #     return result
    #
    # def inquire(self, pred_name: str, arguments: list, relative_arg: int = 0):
    #     if self.open_inquiry is not None:
    #         rospy.logwarn("Last inquiry was not answered yet. Abort.")
    #         return False
    #
    #     self._prepare_data(pred_name, relative_arg)
    #     rule_data = self.data[pred_name]
    #
    #     sample = self.pdm.take_snapshot(arguments)
    #     for i in range(3):
    #         sample.iloc[:, rule_data.relative_arg_selector[i::3]] = sample.iloc[
    #             :, rule_data.relative_arg_selector[i::3]
    #         ].subtract(sample.iloc[:, rule_data.relative_arg_selector[i]], axis=0)
    #
    #     maximum_diff = -1
    #     maximum_rule = ""
    #     lower_values, maximum_diff, maximum_rule = self._get_values_from_rules(
    #         rule_data.lower_rules_forall,
    #         sample,
    #         maximum_diff,
    #         maximum_rule,
    #         rule_data.rules_inquired,
    #     )
    #     upper_values, maximum_diff, maximum_rule = self._get_values_from_rules(
    #         rule_data.upper_rules_forall,
    #         sample,
    #         maximum_diff,
    #         maximum_rule,
    #         rule_data.rules_inquired,
    #     )
    #
    #     # Generate sample that we want labeled
    #     axis_labels = ["_x", "_y", "_z"]
    #     axis_samples = np.array([0.0, 0.0, 0.0])
    #     for i in range(3):
    #         if axis_labels[i] in maximum_rule:
    #             if "<" in maximum_rule:
    #                 lb_idx = list(upper_values[i].index).index(maximum_rule)
    #                 lb_com = upper_values[i].loc[maximum_rule, "abs"]
    #                 ub_com = (
    #                     upper_values[i].iloc[lb_idx + 1, 1]
    #                     if lb_idx < upper_values[i].shape[0]
    #                     else lb_com + 0.3
    #                 )
    #             else:
    #                 ub_idx = list(lower_values[i].index).index(maximum_rule)
    #                 ub_com = lower_values[i].loc[maximum_rule, "abs"]
    #                 lb_com = (
    #                     lower_values[i].iloc[ub_idx + 1, 1]
    #                     if ub_idx < lower_values[i].shape[0]
    #                     else ub_com - 0.3
    #                 )
    #         else:
    #             ub_com = upper_values[i].iloc[0, 1]
    #             lb_com = lower_values[i].iloc[0, 1]
    #         axis_samples[i] = np.random.uniform(lb_com, ub_com)
    #     self.pdm.set_object_position(arguments[relative_arg], axis_samples)
    #     self.open_inquiry = maximum_rule
    #     rospy.loginfo(
    #         f"Testing if rule {maximum_rule} holds. Does the predicate hold in this state?"
    #     )
    #     return True
    #
    # def process_inquire_result(self, label: bool, pred_name: str, arguments: list):
    #     if self.open_inquiry is None:
    #         rospy.logwarn("No inquiry is currently open.")
    #         return False
    #     rule_data = self.data[pred_name]
    #     rule_data.rules_inquired[self.open_inquiry] = not label
    #     if label:
    #         self._set_value_given_rule(rule_data, self.open_inquiry, False)
    #         rospy.loginfo("Thanks for the input. Updated rule set.")
    #         self.pdm.capture_demonstration(pred_name, arguments, label)
    #     self.open_inquiry = None
    #     return True
    #
    # def _load_data(self, name: str):
    #     assert name not in self.data
    #     filename = self._get_file_name(name)
    #     if os.path.isfile(filename):
    #         with open(filename, "rb") as f:
    #             content = pickle.load(f)
    #             self.data[name] = content[0]
    #         return True
    #     return False
    #
    # def _save_data(self):
    #     for name in self.data:
    #         filename = self._get_file_name(name)
    #         content = (self.data[name],)
    #         with open(filename, "wb") as f:
    #             pickle.dump(content, f)
    #     print("Rule data saved")
    #
    # def _get_file_name(self, name):
    #     return os.path.join(self.pdm.pred_dir, f"{name}_rules.pkl")
    #
    # @staticmethod
    # def _set_value_given_rule(rule_data: RuleData, rule: str, value: bool):
    #     axis_labels = ["_x", "_y", "_z"]
    #     axis_idx = None
    #     for i in range(3):
    #         if axis_labels[i] in rule:
    #             axis_idx = i
    #             break
    #     if "<" in rule:
    #         rule_data.upper_rules_forall[axis_idx].loc[rule] = value
    #     else:
    #         rule_data.lower_rules_forall[axis_idx].loc[rule] = value
    #
    # @staticmethod
    # def _get_value_given_rule(rule_data: RuleData, rule: str):
    #     axis_labels = ["_x", "_y", "_z"]
    #     axis_idx = None
    #     for i in range(3):
    #         if axis_labels[i] in rule:
    #             axis_idx = i
    #             break
    #     if "<" in rule:
    #         return rule_data.upper_rules_forall[axis_idx].loc[rule]
    #     else:
    #         return rule_data.lower_rules_forall[axis_idx].loc[rule]
    #
    # def _get_values_from_rules(
    #     self, rules, sample, maximum_diff, maximum_rule, rules_inquired
    # ):
    #     values = {i: None for i in range(3)}
    #     for i in range(3):
    #         this_holding_rules = rules[i][rules[i] == True]
    #
    #         values[i] = pd.DataFrame(index=this_holding_rules.index)
    #         tmp_rel = list(
    #             map(lambda x: self._get_arg_value(x, sample, 0), values[i].index)
    #         )
    #         values[i].insert(0, "rel", tmp_rel)
    #         tmp_abs = list(
    #             map(lambda x: self._get_arg_value(x, sample, 1), values[i].index)
    #         )
    #         values[i].insert(1, "abs", tmp_abs)
    #
    #         values[i] = values[i].subtract(values[i].iloc[:, 0], axis=0)
    #
    #         asc = True if "<" in values[i].index[0] else False
    #         values[i].sort_values(by="abs", inplace=True, ascending=asc)
    #
    #         tmp_diff = values[i].loc[:, "abs"].diff(periods=-1)
    #         if asc:
    #             tmp_diff *= -1
    #         tmp_diff.iloc[-1] = 0
    #         values[i].insert(2, "diff", tmp_diff)
    #
    #         for j, diff in enumerate(values[i].loc[:, "diff"]):
    #             tmp_rule = values[i].index[j]
    #             if tmp_rule not in rules_inquired:
    #                 if diff > maximum_diff:
    #                     maximum_diff = diff
    #                     maximum_rule = tmp_rule
    #                 break
    #     return values, maximum_diff, maximum_rule
    #
    # @staticmethod
    # def _get_arg_value(value_str: str, sample, arg_pos: int):
    #     if "<" in value_str:
    #         value_str_split = value_str.split("<")
    #     elif ">" in value_str:
    #         value_str_split = value_str.split(">")
    #     else:
    #         raise RuntimeError
    #     value_str_select = value_str_split[arg_pos]
    #     return sample.loc[0, value_str_select]
    #
    # def _prepare_data(self, pred_name: str, relative_arg: int):
    #     if pred_name not in self.data:
    #         if not self._load_data(pred_name):
    #             new_rule_data = RuleData()
    #             # new_rule_data.meta_data = self.pdm.get_meta_data(pred_name)
    #             new_rule_data.relative_arg = relative_arg
    #
    #             self.setup_selectors(new_rule_data)
    #
    #             self.data[pred_name] = new_rule_data
    #
    #     self.data[pred_name].processed_data = self.preprocess_data(
    #         self.pdm.get_data(pred_name), self.data[pred_name]
    #     )
    #
    # @staticmethod
    # def setup_selectors(rule_data: RuleData):
    #     if rule_data.meta_data["num_args"] < 2:
    #         return False
    #     non_relative_args = list(range(rule_data.meta_data["num_args"]))
    #     non_relative_args.remove(rule_data.relative_arg)
    #     non_relative_args_selector = list()
    #     for arg in non_relative_args:
    #         non_relative_args_selector.extend(
    #             np.array(range(rule_data.meta_data["num_features"]))
    #             + arg * rule_data.meta_data["num_features"]
    #         )
    #     relative_arg_selector = (
    #         np.array(range(rule_data.meta_data["num_features"]))
    #         + rule_data.relative_arg * rule_data.meta_data["num_features"]
    #     )
    #
    #     origin_arg = non_relative_args[0]
    #
    #     rule_data.relative_arg_selector = relative_arg_selector
    #     rule_data.non_relative_args_selector = non_relative_args_selector
    #     rule_data.non_relative_args = non_relative_args
    #     rule_data.origin_arg = origin_arg
    #
    # @staticmethod
    # def preprocess_data(raw_data: pd.DataFrame, rule_data: RuleData):
    #     origin_arg_selector = (
    #         np.array(range(rule_data.meta_data["num_features"]))
    #         + rule_data.origin_arg * rule_data.meta_data["num_features"]
    #     )
    #
    #     processed_data = raw_data.copy(deep=True)
    #     for i in range(3):
    #         processed_data.iloc[:, i::3] = processed_data.iloc[:, i::3].subtract(
    #             processed_data.iloc[:, origin_arg_selector[i]], axis=0
    #         )
    #     return processed_data
    #
    # @staticmethod
    # def _compare_values(data, rule_data):
    #     # Find upper and lower bounds
    #     gt_relative_arg = {i: pd.DataFrame() for i in range(3)}
    #     lt_relative_arg = {i: pd.DataFrame() for i in range(3)}
    #     for selector in rule_data.relative_arg_selector:
    #         axis_idx = selector % 3
    #         this_non_relative_selector = rule_data.non_relative_args_selector[
    #             axis_idx::3
    #         ]
    #
    #         # Check upper bounds
    #         this_gt_relative_arg = data.iloc[:, this_non_relative_selector].gt(
    #             data.iloc[:, selector], axis="index"
    #         )
    #         renamer = dict()
    #         for col in this_gt_relative_arg.columns:
    #             renamer[col] = f"{data.columns[selector]}<{col}"
    #         this_gt_relative_arg.rename(columns=renamer, inplace=True)
    #         gt_relative_arg[axis_idx] = pd.concat(
    #             [gt_relative_arg[axis_idx], this_gt_relative_arg], axis=1
    #         )
    #
    #         # Check lower bounds
    #         this_lt_relative_arg = data.iloc[:, this_non_relative_selector].lt(
    #             data.iloc[:, selector], axis="index"
    #         )
    #         for col in renamer:
    #             renamer[col] = renamer[col].replace("<", ">")
    #         this_lt_relative_arg.rename(columns=renamer, inplace=True)
    #         lt_relative_arg[axis_idx] = pd.concat(
    #             [lt_relative_arg[axis_idx], this_lt_relative_arg], axis=1
    #         )
    #     return gt_relative_arg, lt_relative_arg
