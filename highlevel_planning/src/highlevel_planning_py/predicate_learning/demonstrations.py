import os
import pickle
import pybullet as pb
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
        pred_dir = os.path.join(basedir, "data", "predicates")
        self.demo_dir = os.path.join(pred_dir, "demonstrations")
        os.makedirs(self.demo_dir, exist_ok=True)

    def capture_demonstration(self, name: str, arguments: list, label: bool):

        meta_file = os.path.join(self.demo_dir, name, "_meta.pkl")
        if os.path.isfile(meta_file):
            with open(meta_file, "rb") as f:
                meta_data = pickle.load(f)
            assert meta_data["num_args"] == len(arguments)
        else:
            meta_data = {"num_args": len(arguments)}
            with open(meta_file, "wb") as f:
                pickle.dump(meta_data, f)

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
            output.write(save_string)

        # Save bullet state
        pb.saveBullet(
            os.path.join(this_demo_dir, "state.bullet"),
            physicsClientId=self.scene.world.client_id,
        )

        return True
