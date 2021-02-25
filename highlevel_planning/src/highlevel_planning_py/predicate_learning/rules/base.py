import os
import pickle


class RulesBase:
    def __init__(self, data_dir):
        pred_dir = os.path.join(data_dir, "predicates")
        self.demo_dir = os.path.join(pred_dir, "demonstrations")
        self.feature_dir = os.path.join(pred_dir, "features")
        self.rule_dir = os.path.join(pred_dir, "rules")
        os.makedirs(self.rule_dir, exist_ok=True)

    def _get_feature_data(self, pred_name):
        feature_file = os.path.join(self.feature_dir, f"{pred_name}.pkl")
        with open(feature_file, "rb") as f:
            feature_data, feature_meta_data = pickle.load(f)
        return feature_data, feature_meta_data
