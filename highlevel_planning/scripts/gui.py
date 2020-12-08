# from tkinter import *
import tkinter as tk
from tkinter import ttk
import rospy
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest


class Application:
    def __init__(self):
        self.root = tk.Tk()
        self.feet = None
        self.meters = None
        self.running = tk.BooleanVar()
        self.pred_name = tk.StringVar()
        self.pred_args = tk.StringVar()

        # rospy.loginfo("Waiting for server ...")
        # get_srv = rospy.ServiceProxy("sim_status", Trigger)
        # get_srv.wait_for_service()
        # res = get_srv(TriggerRequest())
        # self.running.set(res.success)
        # rospy.loginfo("Got sim status, setting up window")

        self.set_srv = rospy.ServiceProxy("sim_switch", SetBool)

        self.setup_window()

    def setup_window(self):
        self.root.title("Simulation Control")

        mainframe = ttk.Frame(self.root, padding="3 3 12 12")
        mainframe.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        ttk.Label(mainframe, text="Simulation status:").grid(
            column=1, row=3, sticky=tk.W
        )
        ttk.Label(mainframe, textvariable=self.running).grid(
            column=2, row=3, sticky=(tk.W,)
        )

        ttk.Button(mainframe, text="Run", command=self._run_sim).grid(column=1, row=1)
        ttk.Button(mainframe, text="Stop", command=self._stop_sim).grid(column=2, row=1)

        ttk.Label(mainframe, text="Predicate name:").grid(column=1, row=4, sticky=tk.W)
        ttk.Entry(mainframe, width=8, textvariable=self.pred_name).grid(
            column=2, row=4, sticky=tk.W
        )
        ttk.Label(mainframe, text="Predicate arguments:").grid(column=1, row=5)
        ttk.Entry(mainframe, width=12, textvariable=self.pred_args).grid(
            column=2, row=5, sticky=tk.W
        )
        ttk.Button(
            mainframe, text="Snapshot +", command=lambda: self._snapshot(True)
        ).grid(column=1, row=6)
        ttk.Button(
            mainframe, text="Snapshot -", command=lambda: self._snapshot(False)
        ).grid(column=2, row=6)

        for child in mainframe.winfo_children():
            child.grid_configure(padx=5, pady=5)

        # feet_entry.focus()
        self.root.bind("<Return>", self._run_sim)

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

    def _snapshot(self, val):
        print(f"hey {val}")
        # self.pred_name.set("")
        # self.pred_args.set("")

    def mainloop(self):
        self.root.mainloop()


if __name__ == "__main__":
    rospy.init_node("hlp_gui")
    app = Application()
    app.mainloop()
