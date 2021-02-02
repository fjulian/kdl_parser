import os
import pickle
from functools import lru_cache
import pandas as pd
import numpy as np
import pybullet as pb

from highlevel_planning_py.sim.world import WorldPybullet
from highlevel_planning_py.sim.scene_planning_1 import ScenePlanning1


class PredicateFeatureManager:
    def __init__(self, basedir):
        self.basedir = basedir
        self.pred_dir = os.path.join(basedir, "data", "predicates")
        self.demo_dir = os.path.join(self.pred_dir, "demonstrations")
        self.feature_dir = os.path.join(self.pred_dir, "features")
        os.makedirs(self.feature_dir, exist_ok=True)

        self.feature_extractors = {
            "com": self._extract_coms,
            "aabb": self._extract_aabbs,
        }

        self.world = None
        self.scene = None

    def extract_features(self, name: str):
        # Load existing data
        filename = os.path.join(self.feature_dir, f"{name}.pkl")
        if os.path.isfile(filename):
            with open(filename, "rb") as f:
                data = pickle.load(f)
        else:
            data = dict()

        # Start simulator
        self.world = WorldPybullet("direct", sleep=False)
        self.scene = ScenePlanning1(self.world, self.basedir, restored_objects=dict())

        this_demo_dir = os.path.join(self.demo_dir, name)
        for demo_id in os.listdir(this_demo_dir):
            if not os.path.isdir(os.path.join(this_demo_dir, demo_id)):
                continue

            # Clear caches
            self._extract_com.cache_clear()
            self._extract_aabb.cache_clear()

            # Load demo meta data
            meta_file_name = os.path.join(this_demo_dir, demo_id, "demo.pkl")
            with open(meta_file_name, "rb") as f:
                meta_data = pickle.load(f)
            _, arguments, label, objects = meta_data

            # Populate simulation
            if objects != self.scene.objects:
                self.world.reset()
                self.scene.set_objects(objects)
                self.scene.add_objects(force_load=True)
            simstate_file_name = os.path.join(this_demo_dir, demo_id, "state.bullet")
            self.world.restore_state(simstate_file_name)

            for feature_name in self.feature_extractors:
                if feature_name not in data:
                    data[feature_name] = pd.DataFrame(columns=["label"])
                if demo_id not in data[feature_name].index:
                    new_data = self.feature_extractors[feature_name](arguments)
                    new_data["label"] = label
                    new_line = pd.DataFrame([new_data], index=[demo_id])
                    data[feature_name] = data[feature_name].append(
                        new_line, verify_integrity=True
                    )

        # Save data
        with open(filename, "wb") as f:
            pickle.dump(data, f)

        # Clean up
        self.world.close()
        self.world = None
        self.scene = None

        return True

    def _extract_coms(self, arguments):
        data = dict()

        # Process arguments
        for i, arg in enumerate(arguments):
            data[f"arg{i}_com"] = self._extract_com(arg)

        # TODO process surrounding objects

        return data

    @lru_cache(maxsize=None)
    def _extract_com(self, obj_name):
        aabb = self._extract_aabb(obj_name)
        com = np.mean(aabb, 0)
        return com

    def _extract_aabbs(self, arguments):
        data = dict()

        # Process arguments
        for i, arg in enumerate(arguments):
            data[f"arg{i}_aabb"] = self._extract_aabb(arg)

        # TODO process surrounding objects

        return data

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
        return aabb
