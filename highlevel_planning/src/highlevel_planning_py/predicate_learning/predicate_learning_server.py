from highlevel_planning_py.tools import run_util
from highlevel_planning_py.sim.scene_planning_1 import ScenePlanning1
from highlevel_planning_py.sim.scene_planning_dw import ScenePlanningDW

from highlevel_planning_py.predicate_learning.demonstrations import (
    PredicateDemonstrationManager,
)
from highlevel_planning_py.predicate_learning.features import PredicateFeatureManager
from highlevel_planning_py.predicate_learning.rules import RuleDataManager
from highlevel_planning_py.predicate_learning.svm_experiments import SVMRules


class SimServer:
    def __init__(self, flags, assets_dir, data_dir):
        self.running = False

        # Load existing simulation data if desired
        # savedir = os.path.join(BASEDIR, "data", "sim")
        # objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

        # Load config file
        # cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

        # Create world
        scene_definition = ScenePlanningDW
        scene, self.world = run_util.setup_pybullet_world(
            scene_definition, assets_dir, flags
        )

        # Predicate learning
        self.pdm = PredicateDemonstrationManager(data_dir, scene)
        self.pfm = PredicateFeatureManager(
            data_dir, assets_dir, self.world, scene, scene_definition
        )
        # self.rdm = RuleDataManager(BASEDIR, self.pfm)
        self.rdm = SVMRules(data_dir, self.pfm)
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
