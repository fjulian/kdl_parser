from highlevel_planning_py.sim.scene_planning_1 import ScenePlanning1
from highlevel_planning_py.skills.navigate import SkillNavigate
from highlevel_planning_py.skills.grasping import SkillGrasping
from highlevel_planning_py.skills.placing import SkillPlacing
from highlevel_planning_py.skills.move import SkillMove
from highlevel_planning_py.knowledge.predicates import Predicates
from highlevel_planning_py.tools.config import ConfigYaml
from highlevel_planning_py.tools import run_util
from highlevel_planning_py.learning.predicate_learning import PredicateDataManager
from highlevel_planning.srv import Snapshot, SnapshotResponse

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import tkinter as tk
import rospy
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class SimServer:
    def __init__(self):
        self.running = False

        # Load existing simulation data if desired
        savedir = os.path.join(BASEDIR, "data", "sim")
        objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

        # Load config file
        cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

        # Create world
        scene, self.world = run_util.setup_pybullet_world(
            ScenePlanning1, BASEDIR, savedir, objects, args
        )

        # Predicate learning
        self.pdm = PredicateDataManager(BASEDIR, scene)

        # GUI services
        self.run_srv = rospy.Service("sim_switch", SetBool, self._set_run_callback)
        self.status_srv = rospy.Service("sim_status", Trigger, self._get_run_callback)
        self.snapshot_srv = rospy.Service(
            "sim_snapshot", Snapshot, self._trigger_snapshot_callback
        )

    def _set_run_callback(self, req):
        self.running = req.data
        res = SetBoolResponse(success=self.running)
        return res

    def _get_run_callback(self, req):
        res = TriggerResponse(success=self.running)
        return res

    def _trigger_snapshot_callback(self, req):
        pred_args = self._process_args(req.pred_args)
        success = self.pdm.capture_demonstration(req.pred_name, pred_args, req.label)
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
