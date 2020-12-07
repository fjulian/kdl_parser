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

        rospy.loginfo("Waiting for server ...")
        get_srv = rospy.ServiceProxy("sim_status", Trigger)
        get_srv.wait_for_service()
        res = get_srv(TriggerRequest())
        self.running.set(res.success)
        rospy.loginfo("Got sim status, setting up window")

        self.set_srv = rospy.ServiceProxy("sim_switch", SetBool)

        self.setup_window()

    def setup_window(self):
        self.root.title("Simulation Control")

        mainframe = ttk.Frame(self.root, padding="3 3 12 12")
        mainframe.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # self.feet = tk.StringVar()
        # feet_entry = ttk.Entry(mainframe, width=7, textvariable=self.feet)
        # feet_entry.grid(column=2, row=1, sticky=(tk.W, tk.E))
        #
        # self.meters = tk.StringVar()
        # ttk.Label(mainframe, textvariable=self.meters).grid(
        #     column=2, row=2, sticky=(tk.W, tk.E)
        # )

        ttk.Label(mainframe, textvariable=self.running).grid(
            column=1, row=2, sticky=(tk.W,)
        )

        ttk.Button(mainframe, text="Run", command=self._run_sim).grid(
            column=1, row=1, sticky=tk.W
        )
        ttk.Button(mainframe, text="Stop", command=self._stop_sim).grid(
            column=2, row=1, sticky=tk.W
        )

        # ttk.Label(mainframe, text="feet").grid(column=3, row=1, sticky=tk.W)
        # ttk.Label(mainframe, text="is equivalent to").grid(column=1, row=2, sticky=tk.E)
        # ttk.Label(mainframe, text="meters").grid(column=3, row=2, sticky=tk.W)

        for child in mainframe.winfo_children():
            child.grid_configure(padx=5, pady=5)

        # feet_entry.focus()
        self.root.bind("<Return>", self._run_sim)

    def calculate(self, *args):
        try:
            value = float(self.feet.get())
            self.meters.set(int(0.3048 * value * 10000.0 + 0.5) / 10000.0)
        except ValueError:
            pass

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

    def mainloop(self):
        self.root.mainloop()


if __name__ == "__main__":
    rospy.init_node("hlp_gui")
    app = Application()
    app.mainloop()
