import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from highlevel_planning_interface_ros2.msg import HlpGuiCommand
from highlevel_planning_interface_ros2.srv import Snapshot

from highlevel_planning_py.predicate_learning.gui import Application


def call_wait_result(node, service, req):
    fut = service.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    return fut.result()


class ApplicationROS2(Application, Node):
    def __init__(self):
        Application.__init__(self)
        Node.__init__(self, "predicate_learning_gui")
        self.logger = self.get_logger()

        self.logger.info("Waiting for server ...")
        get_srv = self.create_client(Trigger, "sim_status")
        get_srv.wait_for_service()
        res = call_wait_result(self, get_srv, Trigger.Request())
        self.running.set(res.success)
        self.logger.info("Got sim status, setting up window")

        self.set_srv = self.create_client(SetBool, "sim_switch")
        self.trigger_srv = self.create_client(Snapshot, "sim_snapshot")

        self.confirm_button = None
        self.deny_button = None
        self.setup_window()

    def _run_sim(self, *args):
        self.logger.info("Sent run command")
        req = SetBool.Request(data=True)
        res = call_wait_result(self, self.set_srv, req)
        if not res.success:
            raise RuntimeError
        self.running.set(True)
        self.root.bind("<Return>", self._stop_sim)

    def _stop_sim(self, *args):
        self.logger.info("Sent stop command")
        req = SetBool.Request(data=False)
        res = call_wait_result(self, self.set_srv, req)
        if res.success:
            raise RuntimeError
        self.running.set(False)
        self.root.bind("<Return>", self._run_sim)

    def _snapshot(self, cmd, label=True):
        if cmd == 4:
            if self.confirm_button["state"] == "normal":
                self.logger.warn(
                    "Previous inquiry still open, please reply to that one first."
                )
                return
            else:
                self.confirm_button["state"] = "normal"
                self.deny_button["state"] = "normal"
        elif cmd == 5:
            self.confirm_button["state"] = "disabled"
            self.deny_button["state"] = "disabled"

        cmd_msg = HlpGuiCommand()
        cmd_msg.cmd = cmd
        req = Snapshot.Request(
            command=cmd_msg,
            pred_name=self.pred_name.get(),
            pred_args=self.pred_args.get(),
            relative_arg=self.rel_arg.get(),
            label=label,
        )
        res = call_wait_result(self, self.trigger_srv, req)
        self.logger.info("Sent command")


def main(args=None):
    rclpy.init(args=args)
    app = ApplicationROS2()
    app.mainloop()


if __name__ == "__main__":
    main()
