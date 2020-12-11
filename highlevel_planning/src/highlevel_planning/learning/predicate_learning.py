import os
import pickle
import pandas as pd


class PredicateDataManager:
    def __init__(self, basedir):
        self.data = dict()
        self.pred_dir = os.path.join(basedir, "data", "predicates")
        os.makedirs(self.pred_dir, exist_ok=True)

    def extract_features(self):
        pass

    def take_snapshot(self, name: str, raw_args: str, label: bool):
        if name not in self.data:
            res = self._load_pred_data(name)
            if not res:
                self.data[name] = pd.DataFrame()

        arguments = raw_args.split(",")
        for i, a in enumerate(arguments):
            arguments[i] = a.strip()

        return True

    def _load_pred_data(self, name: str):
        assert name not in self.data
        filename = os.path.join(self.pred_dir, f"{name}.pkl")
        if os.path.isfile(filename):
            with open(filename, "rb") as f:
                self.data[name] = pickle.load(f)
            return True
        return False

    def _save_pred_data(self, name: str):
        pass
