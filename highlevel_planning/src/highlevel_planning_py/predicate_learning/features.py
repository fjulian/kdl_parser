import os
import pickle
from functools import cache
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

        self.world = None
        self.scene = None

    def extract_features(self, name: str):
        feature_extractor = {"com": self._extract_com, "aabb": self._extract_aabb}
        this_feature_dir = os.path.join(self.feature_dir, name)
        os.makedirs(this_feature_dir, exist_ok=True)

        data = dict()

        # Load existing data
        for feature_name in feature_extractor:
            filename = os.path.join(this_feature_dir, f"{feature_name}.pkl")
            if os.path.isfile(filename):
                with open(filename, "rb") as f:
                    data[feature_name] = pickle.load(f)
            else:
                data[feature_name] = pd.DataFrame(columns=["predicate_name", "label"])

        # Start simulator
        self.world = WorldPybullet("direct", sleep=False)
        self.scene = ScenePlanning1(self.world, self.basedir, restored_objects=dict())

        this_demo_dir = os.path.join(self.demo_dir, name)
        for demo_id in os.listdir(this_demo_dir):
            if not os.path.isdir(os.path.join(this_demo_dir, demo_id)):
                continue

            for feature_name in feature_extractor:
                if demo_id not in data[feature_name].index:
                    # Load demo meta data
                    meta_file_name = os.path.join(this_demo_dir, demo_id, "demo.pkl")
                    with open(meta_file_name, "rb") as f:
                        meta_data = pickle.load(f)
                    _, arguments, label, objects = meta_data

                    if objects != self.scene.objects:
                        self.world.reset()
                        self.scene.set_objects(objects)
                        self.scene.add_objects()
                    simstate_file_name = os.path.join(
                        this_demo_dir, demo_id, "state.bullet"
                    )
                    self.world.restore_state(simstate_file_name)

                    new_data, new_labels = feature_extractor[feature_name](arguments)
                    new_line = pd.DataFrame(
                        new_data, index=[demo_id], columns=new_labels
                    )
                    data[feature_name] = data[feature_name].append(new_line)

        # Check cache effectiveness
        print(f"CoM cache info: {self._extract_com.cache_info()}")
        print(f"aabb cache info: {self._extract_aabb.cache_info()}")

        # Clean up
        self.world.close()
        self.world = None
        self._extract_com.cache_clear()
        self._extract_aabb.cache_clear()

    def _extract_coms(self, arguments):
        labels = list()
        data = list()

        # Process arguments
        for i, arg in enumerate(arguments):
            data.append(self._extract_com(arg))
            labels.append(f"arg{i}_com")

        # TODO process surrounding objects

        return data, labels

    @cache
    def _extract_com(self, obj_name):
        aabb = self._extract_aabb(obj_name)
        com = np.mean(aabb, 0)
        return com

    def _extract_aabbs(self, arguments):
        labels = list()
        data = list()

        # Process arguments
        for i, arg in enumerate(arguments):
            data.append(self._extract_aabb(arg))
            labels.append(f"arg{i}_aabb")

        # TODO process surrounding objects

        return data, labels

    @cache
    def _extract_aabb(self, obj_name):
        obj_uid = self.scene.objects[obj_name].model.uid
        link2idx = self.scene.objects[obj_name].model.link_name_to_index
        aabb = np.array(pb.getAABB(obj_uid))
        for link in link2idx:
            tmp = np.array(pb.getAABB(obj_uid, link2idx[link]))
            aabb[0, :] = np.minimum(aabb[0, :], tmp[0, :])
            aabb[1, :] = np.maximum(aabb[1, :], tmp[1, :])
        return aabb
