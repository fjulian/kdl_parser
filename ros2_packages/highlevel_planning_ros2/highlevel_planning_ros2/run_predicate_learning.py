import os
import atexit
import threading
import rclpy
from rclpy.node import Node
import ament_index_python as ament_index
from argparse import Namespace

from highlevel_planning_py.tools import run_util
from highlevel_planning_py.predicate_learning.predicate_learning_server import SimServer
from highlevel_planning_interface_ros2.srv import Snapshot
from highlevel_planning_interface_ros2.msg import ManipulationCmd


from std_srvs.srv import SetBool, Trigger


class SimServerROS2(SimServer, Node):
    def __init__(self, assets_dir_, data_dir_):
        Node.__init__(self, "simserver_ros2")
        self.logger = self.get_logger()

        self.declare_parameters(
            "",
            [
                ("method", "gui"),
                ("reuse_objects", False),
                ("sleep", True),
                ("non_interactive", False),
                ("no_seed", False),
            ],
        )
        method = self.get_parameter("method").value
        reuse_objects = self.get_parameter("reuse_objects").value
        sleep = self.get_parameter("sleep").value
        non_interactive = self.get_parameter("non_interactive").value
        no_seed = self.get_parameter("no_seed").value
        flags = Namespace(
            method=method,
            reuse_objects=reuse_objects,
            sleep=sleep,
            non_interactive=non_interactive,
            no_seed=no_seed,
        )

        SimServer.__init__(self, flags, assets_dir_, data_dir_)

        self.logger.info("Started simulation with the following objects")

        # GUI services
        self.run_srv = self.create_service(
            SetBool, "sim_switch", self._set_run_callback
        )
        self.status_srv = self.create_service(
            Trigger, "sim_status", self._get_run_callback
        )
        self.snapshot_srv = self.create_service(
            Snapshot, "sim_snapshot", self._snapshot_callback
        )

        # Subscribers
        self.create_subscription(
            ManipulationCmd, "move_manually", self._move_manual_callback, 10
        )

    def _print(self, msg):
        self.logger.info(msg)

    def _set_run_callback(self, req, res):
        self.running = req.data
        res.success = self.running
        return res

    def _get_run_callback(self, req, res):
        res.success = self.running
        return res

    def _snapshot_callback(self, req, res):
        cmd = req.command.cmd
        pred_args = self._process_args(req.pred_args)
        if cmd == 0:
            success = self.pdm.capture_demonstration(
                req.pred_name, pred_args, req.label
            )
            self.logger.info("Captured demonstration: {}".format(success))
        elif cmd == 1:
            success = self.pfm.extract_demo_features(req.pred_name)
            self.logger.info("Extracted features: {}".format(success))
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
        res.success = success
        return res

    def _move_manual_callback(self, msg):
        ret = self._move_manual(msg.target_name, msg.translation, msg.rotation)
        if ret is not None:
            self.logger.warn(ret)

    def thread_loop(self, quit_event):
        rate = self.create_rate(240.0)
        self.logger.info("Starting simulation loop")
        while True:
            if quit_event.is_set():
                self.logger.info("Bye")
                return
            self.loop()
            rate.sleep()


def exit_handler(quit_event):
    quit_event.set()


def main(args=None):
    rclpy.init(args=args)

    data_dir = os.path.join(os.path.expanduser("~"), "Data", "highlevel_planning")
    os.makedirs(data_dir, exist_ok=True)

    asset_dir = ament_index.get_package_prefix("dishwasher_description")
    asset_dir = os.path.join(asset_dir, "share", "dishwasher_description", "assets")

    app = SimServerROS2(asset_dir, data_dir)

    quit_event_ = threading.Event()
    atexit.register(exit_handler, quit_event=quit_event_)
    sim_thread = threading.Thread(target=app.thread_loop, args=(quit_event_,))
    sim_thread.start()
    rclpy.spin(app)


if __name__ == "__main__":
    main()
