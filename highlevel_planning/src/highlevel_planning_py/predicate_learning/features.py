import os
import pickle
from functools import lru_cache
import pandas as pd
import numpy as np
import pybullet as pb
from highlevel_planning_py.sim.world import WorldPybullet


class PredicateFeatureManager:
    def __init__(
        self, data_dir, assets_dir, outer_world, outer_scene, inner_scene_definition
    ):
        self.assets_dir = assets_dir
        pred_dir = os.path.join(data_dir, "predicates")
        self.demo_dir = os.path.join(pred_dir, "demonstrations")
        self.feature_dir = os.path.join(pred_dir, "features")
        os.makedirs(self.feature_dir, exist_ok=True)

        self.feature_extractors = {"com": self._extract_com, "aabb": self._extract_aabb}

        self.outer_world = outer_world
        self.outer_scene = outer_scene
        self.inner_scene_definition = inner_scene_definition

        self.world = None
        self.scene = None

    def extract_demo_features(self, name: str):
        """
        Extracts features for all demonstrations that were previously saved to disk.
        """

        # Load existing data
        filename = os.path.join(self.feature_dir, f"{name}.pkl")
        if os.path.isfile(filename):
            with open(filename, "rb") as f:
                data, meta_data = pickle.load(f)
        else:
            data = dict()
            meta_data = dict()
            meta_data["demos_processed"] = set()

        # Start simulator
        self.world = WorldPybullet("direct", sleep=False)
        self.scene = self.inner_scene_definition(
            self.world, self.assets_dir, restored_objects=dict()
        )

        this_demo_dir = os.path.join(self.demo_dir, name)
        for demo_id in os.listdir(this_demo_dir):
            if not os.path.isdir(os.path.join(this_demo_dir, demo_id)):
                continue

            # Skip if this was already processed
            if demo_id in meta_data["demos_processed"]:
                continue

            # Clear caches
            self._extract_com.cache_clear()
            self._extract_aabb.cache_clear()

            # Load demo meta data
            meta_file_name = os.path.join(this_demo_dir, demo_id, "demo.pkl")
            with open(meta_file_name, "rb") as f:
                demo_meta_data = pickle.load(f)
            _, arguments, label, objects = demo_meta_data

            # Populate simulation
            if objects != self.scene.objects:
                self.world.reset()
                self.scene.set_objects(objects)
                self.scene.add_objects(force_load=True)
            simstate_file_name = os.path.join(this_demo_dir, demo_id, "state.bullet")
            self.world.restore_state(simstate_file_name)

            for arg_idx, arg in enumerate(arguments):
                if f"arg{arg_idx}" not in data:
                    data[f"arg{arg_idx}"] = pd.DataFrame()

                new_row = pd.DataFrame({"label": label}, index=[demo_id])

                for feature_name in self.feature_extractors:
                    new_data, new_labels = self.feature_extractors[feature_name](arg)
                    if len(new_data) > 0:
                        new_data = np.squeeze(new_data.reshape((1, -1)))
                        new_data = pd.DataFrame(
                            [new_data],
                            columns=[
                                f"arg{arg_idx}_{feature_name}_{sfx}"
                                for sfx in new_labels
                            ],
                            index=[demo_id],
                        )
                        new_row = new_row.join(new_data)

                data[f"arg{arg_idx}"] = data[f"arg{arg_idx}"].append(
                    new_row, verify_integrity=True
                )

            # Mark this demo as processed
            meta_data["demos_processed"].add(demo_id)

        # Save data
        with open(filename, "wb") as f:
            pickle.dump((data, meta_data), f)

        # Clean up
        self.world.close()
        self.world = None
        self.scene = None

        return True

    def extract_outer_features(self, arguments):
        self.world = self.outer_world
        self.scene = self.outer_scene

        data = dict()
        for arg_idx, arg in enumerate(arguments):
            data[f"arg{arg_idx}"] = pd.DataFrame()

            for feature_name in self.feature_extractors:
                new_data, new_labels = self.feature_extractors[feature_name](arg)
                if len(new_data) > 0:
                    new_data = np.squeeze(new_data.reshape((1, -1)))
                    new_data = pd.DataFrame(
                        [new_data],
                        columns=[
                            f"arg{arg_idx}_{feature_name}_{sfx}" for sfx in new_labels
                        ],
                        index=[0],
                    )
                    data[f"arg{arg_idx}"] = data[f"arg{arg_idx}"].join(
                        new_data, how="outer"
                    )

        self.world = None
        self.scene = None
        return data

    @lru_cache(maxsize=None)
    def _extract_com(self, obj_name):
        aabb, _ = self._extract_aabb(obj_name)
        com = np.mean(aabb, 0)
        labels = ["x", "y", "z"]
        return com, labels

    @lru_cache(maxsize=None)
    def _extract_aabb(self, obj_name):
        obj_uid = self.scene.objects[obj_name].model.uid
        link2idx = self.scene.objects[obj_name].model.link_name_to_index
        aabb = np.array(pb.getAABB(obj_uid, physicsClientId=self.world.client_id))
        for link in link2idx:
            tmp = np.array(
                pb.getAABB(
                    obj_uid, link2idx[link], physicsClientId=self.world.client_id
                )
            )
            aabb[0, :] = np.minimum(aabb[0, :], tmp[0, :])
            aabb[1, :] = np.maximum(aabb[1, :], tmp[1, :])
        labels = ["x_min", "y_min", "z_min", "x_max", "y_max", "z_max"]
        return aabb, labels
