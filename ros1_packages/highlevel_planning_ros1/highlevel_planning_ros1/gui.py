import rospy
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest
from highlevel_planning.msg import HlpGuiCommand
from highlevel_planning.srv import Snapshot, SnapshotRequest

from highlevel_planning_py.predicate_learning.gui import Application


class ApplicationROS1(Application):
    def __init__(self):
        Application.__init__(self)

        rospy.loginfo("Waiting for server ...")
        get_srv = rospy.ServiceProxy("sim_status", Trigger)
        get_srv.wait_for_service()
        res = get_srv(TriggerRequest())
        self.running.set(res.success)
        rospy.loginfo("Got sim status, setting up window")

        self.set_srv = rospy.ServiceProxy("sim_switch", SetBool)
        self.trigger_srv = rospy.ServiceProxy("sim_snapshot", Snapshot)

        self.confirm_button = None
        self.deny_button = None
        self.setup_window()

    def _run_sim(self, *args):
        print("Sent run command")
        res = self.set_srv(SetBoolRequest(data=True))
        if not res.success:
            raise RuntimeError
        self.running.set(True)
        self.root.bind("<Return>", self._stop_sim)

    def _stop_sim(self, *args):
        print("Sent stop command")
        res = self.set_srv(SetBoolRequest(data=False))
        if res.success:
            raise RuntimeError
        self.running.set(False)
        self.root.bind("<Return>", self._run_sim)

    def _snapshot(self, cmd, label=True):
        if cmd == 4:
            if self.confirm_button["state"] == "normal":
                rospy.logwarn(
                    "Previous inquiry still open, please reply to that one first."
                )
                return
            else:
                self.confirm_button["state"] = "normal"
                self.deny_button["state"] = "normal"
        elif cmd == 5:
            self.confirm_button["state"] = "disabled"
            self.deny_button["state"] = "disabled"

        cmd_msg = HlpGuiCommand(cmd)
        res = self.trigger_srv(
            SnapshotRequest(
                command=cmd_msg,
                pred_name=self.pred_name.get(),
                pred_args=self.pred_args.get(),
                relative_arg=self.rel_arg.get(),
                label=label,
            )
        )

        rospy.loginfo("Sent command")


if __name__ == "__main__":
    rospy.init_node("hlp_gui")
    app = ApplicationROS1()
    app.mainloop()
