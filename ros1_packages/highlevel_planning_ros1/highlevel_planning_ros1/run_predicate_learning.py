import os
import rospy

from highlevel_planning_py.tools import run_util
from highlevel_planning_py.predicate_learning.predicate_learning_server import SimServer
from highlevel_planning_ros1.srv import Snapshot, SnapshotResponse


from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class SimServerROS1(SimServer):
    def __init__(self, flags_):
        super().__init__(flags_)

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


if __name__ == "__main__":
    flags = run_util.parse_arguments()
    rospy.init_node("hlp_simulator")
    app = SimServerROS1(flags)
    rate = rospy.Rate(240)
    while not rospy.is_shutdown():
        app.loop()
        rate.sleep()
