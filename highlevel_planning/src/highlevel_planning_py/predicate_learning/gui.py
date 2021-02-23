import tkinter as tk
from tkinter import ttk


class Application:
    def __init__(self):
        self.root = tk.Tk()
        self.feet = None
        self.meters = None
        self.running = tk.BooleanVar()
        self.pred_name = tk.StringVar()
        self.pred_args = tk.StringVar()
        self.rel_arg = tk.IntVar()

        self.confirm_button = None
        self.deny_button = None
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
        ttk.Label(mainframe, text="Relative argument:").grid(column=1, row=6)
        ttk.Entry(mainframe, width=12, textvariable=self.rel_arg).grid(
            column=2, row=6, sticky=tk.W
        )
        ttk.Button(
            mainframe, text="+ Demonstration", command=lambda: self._snapshot(0, True)
        ).grid(column=1, row=7)
        ttk.Button(
            mainframe, text="- Demonstration", command=lambda: self._snapshot(0, False)
        ).grid(column=2, row=7)
        ttk.Button(
            mainframe, text="Extract features", command=lambda: self._snapshot(1)
        ).grid(column=1, row=8)
        ttk.Button(
            mainframe, text="Build rules", command=lambda: self._snapshot(2)
        ).grid(column=2, row=8)
        ttk.Button(mainframe, text="Classify", command=lambda: self._snapshot(3)).grid(
            column=3, row=8
        )
        ttk.Button(mainframe, text="Inquire", command=lambda: self._snapshot(4)).grid(
            column=1, row=9
        )
        self.confirm_button = ttk.Button(
            mainframe,
            text="Confirm",
            state=tk.DISABLED,
            command=lambda: self._snapshot(5, True),
        )
        self.confirm_button.grid(column=2, row=9)
        self.deny_button = ttk.Button(
            mainframe,
            text="Deny",
            state=tk.DISABLED,
            command=lambda: self._snapshot(5, False),
        )
        self.deny_button.grid(column=3, row=9)

        for child in mainframe.winfo_children():
            child.grid_configure(padx=5, pady=5)

        # feet_entry.focus()
        self.root.bind("<Return>", self._run_sim)

    def _run_sim(self, *args):
        raise NotImplementedError

    def _stop_sim(self, *args):
        raise NotImplementedError

    def _snapshot(self, cmd, label=True):
        raise NotImplementedError

    def mainloop(self):
        self.root.mainloop()
