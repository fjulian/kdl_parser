import os
import pickle
import pandas as pd
from sklearn.svm import SVC
import seaborn as sns
import matplotlib.pyplot as plt


def dir_levels_up(filepath, num_levels_up):
    res = filepath
    for _ in range(num_levels_up):
        res = os.path.dirname(res)
    return res


class SVMRules:
    def __init__(self, data_dir, feature_manager):
        self.pfm = feature_manager

        pred_dir = os.path.join(data_dir, "data", "predicates")
        self.demo_dir = os.path.join(pred_dir, "demonstrations")
        self.feature_dir = os.path.join(pred_dir, "features")
        # self.rule_dir = os.path.join(pred_dir, "rules_svm")
        # os.makedirs(self.rule_dir, exist_ok=True)
        self.clfs = dict()

    def build_rules(self, pred_name):
        if pred_name not in self.clfs:
            self.clfs[pred_name] = SVC(kernel="linear")

        # Load features
        feature_file = os.path.join(self.feature_dir, f"{pred_name}.pkl")
        with open(feature_file, "rb") as f:
            feature_data, _ = pickle.load(f)

        merged_data = self._merge_data(feature_data)
        merged_data = merged_data.drop(columns="label_r")

        self.clfs[pred_name].fit(merged_data.iloc[:, 1:], merged_data.iloc[:, 0])

        # Visualize rules
        sns.pairplot(merged_data, hue="label", palette="bright")
        print("bla")

        return True

    def classify(self, pred_name, arguments):
        features = self.pfm.extract_outer_features(arguments)
        merged_features = self._merge_data(features)
        prediction = self.clfs[pred_name].predict(merged_features)[0]
        print("Prediction is {}".format(prediction))
        return prediction

    @staticmethod
    def _merge_data(feature_dict):
        merged_data = pd.DataFrame()
        for arg in feature_dict:
            merged_data = merged_data.join(feature_dict[arg], how="outer", rsuffix="_r")
        return merged_data


if __name__ == "__main__":
    data_dir_ = os.path.join(os.path.expanduser("~"), "Data", "highlevel_planning")
    os.makedirs(data_dir_, exist_ok=True)
    svmr = SVMRules(data_dir_, None)
    svmr.build_rules("on")
