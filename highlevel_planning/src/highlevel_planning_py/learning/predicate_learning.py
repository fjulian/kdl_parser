import os
import pickle
import pandas as pd
import atexit


class PredicateDataManager:
    def __init__(self, basedir):
        self.data = dict()
        self.meta_data = dict()
        self.pred_dir = os.path.join(basedir, "data", "predicates")
        os.makedirs(self.pred_dir, exist_ok=True)
        atexit.register(self._save_pred_data)

    def extract_features(self):
        pass

    def take_snapshot(self, name: str, raw_args: str, label: bool):
        data_exists = True
        if name not in self.data:
            data_exists = self._load_pred_data(name)
            if not data_exists:
                self.data[name] = pd.DataFrame()
                self.meta_data[name] = dict.fromkeys(["num_args"])

        arguments = raw_args.split(",")
        for i, a in enumerate(arguments):
            arguments[i] = a.strip()

        if data_exists:
            assert self.meta_data[name]["num_args"] == len(arguments)
        else:
            self.meta_data[name]["num_args"] = len(arguments)

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
