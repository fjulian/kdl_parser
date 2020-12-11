from highlevel_planning.sim.scene_planning_1 import ScenePlanning1
from highlevel_planning.skills.navigate import SkillNavigate
from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.skills.placing import SkillPlacing
from highlevel_planning.skills.move import SkillMove
from highlevel_planning.knowledge.predicates import Predicates
from highlevel_planning.tools.config import ConfigYaml
from highlevel_planning.tools import run_util
from highlevel_planning.learning.predicate_learning import PredicateDataManager
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
        self.pdm = PredicateDataManager(BASEDIR)

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
        success = self.pdm.take_snapshot(req.pred_name, req.pred_args, req.label)
        res = SnapshotResponse(success=success)
        return res

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
