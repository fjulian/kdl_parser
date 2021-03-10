import os
import pandas as pd
from sklearn.ensemble import AdaBoostClassifier
from highlevel_planning_py.predicate_learning.rules.base import RulesBase


class BoostingRules(RulesBase):
    def __init__(self, data_dir, feature_manager):
        RulesBase.__init__(self, data_dir)
        self.pfm = feature_manager
        self.clfs = dict()

    def build_rules(self, pred_name):
        if pred_name not in self.clfs:
            self.clfs[pred_name] = AdaBoostClassifier(n_estimators=50)

        # Load features
        feature_data, _ = self._get_feature_data(pred_name)

        merged_data = self._merge_data(feature_data)
        merged_data = merged_data.drop(columns="label_r")

        self.clfs[pred_name].fit(merged_data.iloc[:, 1:], merged_data.iloc[:, 0])

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
    svmr = BoostingRules(data_dir_, None)
    svmr.build_rules("in")
