from highlevel_planning_py.tools import run_util
from highlevel_planning_py.sim.scene_planning_1 import ScenePlanning1

from highlevel_planning_py.predicate_learning.demonstrations import (
    PredicateDemonstrationManager,
)
from highlevel_planning_py.predicate_learning.features import PredicateFeatureManager
from highlevel_planning_py.predicate_learning.rules import RuleDataManager
from highlevel_planning_py.predicate_learning.svm_experiments import SVMRules

import os


BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class SimServer:
    def __init__(self, flags):
        self.running = False

        # Load existing simulation data if desired
        # savedir = os.path.join(BASEDIR, "data", "sim")
        # objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

        # Load config file
        # cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

        # Create world
        scene, self.world = run_util.setup_pybullet_world(
            ScenePlanning1, BASEDIR, flags
        )

        # Predicate learning
        self.pdm = PredicateDemonstrationManager(BASEDIR, scene)
        self.pfm = PredicateFeatureManager(BASEDIR, self.world, scene)
        # self.rdm = RuleDataManager(BASEDIR, self.pfm)
        self.rdm = SVMRules(BASEDIR, self.pfm)
        # self.pl = PredicateLearner(self.pdm)

    @staticmethod
    def _process_args(raw_args):
        arguments = raw_args.split(",")
        for i, a in enumerate(arguments):
            arguments[i] = a.strip()
        return arguments

    def loop(self):
        if self.running:
            self.world.step_one()
