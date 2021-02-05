from highlevel_planning_py.tools.config import ConfigYaml
from highlevel_planning_py.tools import run_util
from highlevel_planning_py.sim.scene_planning_1 import ScenePlanning1

from highlevel_planning_py.predicate_learning.demonstrations import (
    PredicateDemonstrationManager,
)
from highlevel_planning_py.predicate_learning.features import PredicateFeatureManager
from highlevel_planning_py.predicate_learning.rules import RuleDataManager
from highlevel_planning.srv import Snapshot, SnapshotResponse

import os
import rospy
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class SimServer:
    def __init__(self):
        self.running = False

        # Load existing simulation data if desired
        # savedir = os.path.join(BASEDIR, "data", "sim")
        # objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

        # Load config file
        # cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

        # Create world
        scene, self.world = run_util.setup_pybullet_world(ScenePlanning1, BASEDIR, args)

        # Predicate learning
        self.pdm = PredicateDemonstrationManager(BASEDIR, scene)
        self.pfm = PredicateFeatureManager(BASEDIR, self.world, scene)
        self.rdm = RuleDataManager(BASEDIR, self.pfm)
        # self.pl = PredicateLearner(self.pdm)

        # GUI services
        self.run_srv = rospy.Service("sim_switch", SetBool, self._set_run_callback)
        self.status_srv = rospy.Service("sim_status", Trigger, self._get_run_callback)
        self.snapshot_srv = rospy.Service(
            "sim_snapshot", Snapshot, self._snapshot_callback
        )

    def _set_run_callback(self, req):
        self.running = req.data
        res = SetBoolResponse(success=self.running)
        return res

    def _get_run_callback(self, req):
        res = TriggerResponse(success=self.running)
        return res

    def _snapshot_callback(self, req):
        cmd = req.command.cmd
        pred_args = self._process_args(req.pred_args)
        if cmd == 0:
            success = self.pdm.capture_demonstration(
                req.pred_name, pred_args, req.label
            )
            rospy.loginfo(f"Captured demonstration: {success}")
        elif cmd == 1:
            success = self.pfm.extract_demo_features(req.pred_name)
            rospy.loginfo(f"Extracted features: {success}")
        elif cmd == 2:
            success = self.rdm.build_rules(req.pred_name)
        elif cmd == 3:
            success = self.rdm.classify(req.pred_name, pred_args)
        elif cmd == 4:
            success = self.pl.inquire(
                req.pred_name, pred_args, relative_arg=req.relative_arg
            )
        elif cmd == 5:
            success = self.pl.process_inquire_result(
                req.label, req.pred_name, pred_args
            )
        else:
            success = False
        res = SnapshotResponse(success=success)
        return res

    @staticmethod
    def _process_args(raw_args):
        arguments = raw_args.split(",")
        for i, a in enumerate(arguments):
            arguments[i] = a.strip()
        return arguments

    def loop(self):
        if self.running:
            self.world.step_one()


if __name__ == "__main__":
    # Command line arguments
    args = run_util.parse_arguments()

    rospy.init_node("hlp_simulator")

    app = SimServer()

    rate = rospy.Rate(240)
    while not rospy.is_shutdown():
        app.loop()
        rate.sleep()
