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
        self.moving_object_name = tk.StringVar()

        self.confirm_button = None
        self.deny_button = None
        self.progress_bar = None
        self.setup_window()

    def setup_window(self):
        self.root.title("Simulation Control")

        mainframe = ttk.Frame(self.root, padding="3 3 12 12")
        mainframe.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        ttk.Button(mainframe, text="Run", command=self._run_sim).grid(row=1, column=1)
        ttk.Button(mainframe, text="Stop", command=self._stop_sim).grid(row=1, column=2)
        ttk.Label(mainframe, text="Simulation status:").grid(row=3, column=1)
        ttk.Label(mainframe, textvariable=self.running).grid(row=3, column=2)

        # Demonstration interface
        demo_frame = ttk.Frame(mainframe, borderwidth=5, relief="ridge")
        demo_frame.grid(row=4, column=1, columnspan=2)

        ttk.Label(demo_frame, text="Demonstration Interface").grid(
            column=1, row=0, columnspan=3
        )
        ttk.Label(demo_frame, text="Predicate name:").grid(column=1, row=1, sticky=tk.W)
        ttk.Entry(demo_frame, width=8, textvariable=self.pred_name).grid(
            column=2, row=1, sticky=tk.W
        )
        ttk.Label(demo_frame, text="Predicate arguments:").grid(column=1, row=3)
        ttk.Entry(demo_frame, width=12, textvariable=self.pred_args).grid(
            column=2, row=3, sticky=tk.W
        )
        ttk.Label(demo_frame, text="Relative argument:").grid(
            column=1, row=4, sticky=tk.W
        )
        ttk.Entry(demo_frame, width=12, textvariable=self.rel_arg).grid(
            column=2, row=4, sticky=tk.W
        )
        ttk.Button(
            demo_frame, text="+ Demonstration", command=lambda: self._snapshot(0, True)
        ).grid(column=1, row=5)
        ttk.Button(
            demo_frame, text="- Demonstration", command=lambda: self._snapshot(0, False)
        ).grid(column=2, row=5)
        ttk.Button(
            demo_frame, text="Extract features", command=lambda: self._snapshot(1)
        ).grid(column=1, row=6)
        ttk.Button(
            demo_frame, text="Build rules", command=lambda: self._snapshot(2)
        ).grid(column=2, row=6)
        ttk.Button(demo_frame, text="Classify", command=lambda: self._snapshot(3)).grid(
            column=3, row=6
        )
        ttk.Button(demo_frame, text="Inquire", command=lambda: self._snapshot(4)).grid(
            column=1, row=7
        )
        self.confirm_button = ttk.Button(
            demo_frame,
            text="Confirm",
            state=tk.DISABLED,
            command=lambda: self._snapshot(5, True),
        )
        self.confirm_button.grid(column=2, row=7)
        self.deny_button = ttk.Button(
            demo_frame,
            text="Deny",
            state=tk.DISABLED,
            command=lambda: self._snapshot(5, False),
        )
        self.deny_button.grid(column=3, row=7)
        self.progress_bar = ttk.Progressbar(
            demo_frame, orient=tk.HORIZONTAL, length=200, mode="indeterminate"
        )
        self.progress_bar.grid(row=8, column=1, columnspan=3)

        for child in demo_frame.winfo_children():
            child.grid_configure(padx=5, pady=5)

        # Manipulation interface
        manipulation_frame = ttk.Frame(mainframe, borderwidth=5, relief="ridge")
        manipulation_frame.grid(row=5, column=1, columnspan=2)

        ttk.Label(manipulation_frame, text="Manual Manipulation Interface").grid(
            column=1, row=1, columnspan=2
        )
        ttk.Label(manipulation_frame, text="Target name:").grid(
            column=1, row=2, sticky=tk.W
        )
        ttk.Entry(
            manipulation_frame, width=10, textvariable=self.moving_object_name
        ).grid(column=2, row=2, sticky=tk.W)
        ttk.Label(manipulation_frame, text="Ctrl-Alt-<key> to translate").grid(
            column=1, row=3, columnspan=2
        )
        ttk.Label(manipulation_frame, text="Ctrl-Alt-Shift-<key> to rotate").grid(
            column=1, row=4, columnspan=2
        )

        for child in manipulation_frame.winfo_children():
            child.grid_configure(padx=5, pady=5)

        for child in mainframe.winfo_children():
            child.grid_configure(padx=5, pady=5)

        # Key bindings
        self.root.bind("<Return>", self._run_sim)
        self._bind_key("<Control-Alt-KeyPress-u>", (0, "trans", "fast", -1))
        self._bind_key("<Control-Alt-KeyPress-i>", (0, "trans", "slow", -1))
        self._bind_key("<Control-Alt-KeyPress-o>", (0, "trans", "slow", 1))
        self._bind_key("<Control-Alt-KeyPress-p>", (0, "trans", "fast", 1))
        self._bind_key("<Control-Alt-KeyPress-j>", (1, "trans", "fast", -1))
        self._bind_key("<Control-Alt-KeyPress-k>", (1, "trans", "slow", -1))
        self._bind_key("<Control-Alt-KeyPress-l>", (1, "trans", "slow", 1))
        self._bind_key("<Control-Alt-KeyPress-semicolon>", (1, "trans", "fast", 1))
        self._bind_key("<Control-Alt-KeyPress-m>", (2, "trans", "fast", -1))
        self._bind_key("<Control-Alt-KeyPress-comma>", (2, "trans", "slow", -1))
        self._bind_key("<Control-Alt-KeyPress-period>", (2, "trans", "slow", 1))
        self._bind_key("<Control-Alt-KeyPress-slash>", (2, "trans", "fast", 1))
        self._bind_key("<Control-Alt-Shift-KeyPress-U>", (0, "rot", "fast", -1))
        self._bind_key("<Control-Alt-Shift-KeyPress-I>", (0, "rot", "slow", -1))
        self._bind_key("<Control-Alt-Shift-KeyPress-O>", (0, "rot", "slow", 1))
        self._bind_key("<Control-Alt-Shift-KeyPress-P>", (0, "rot", "fast", 1))
        self._bind_key("<Control-Alt-Shift-KeyPress-J>", (1, "rot", "fast", -1))
        self._bind_key("<Control-Alt-Shift-KeyPress-K>", (1, "rot", "slow", -1))
        self._bind_key("<Control-Alt-Shift-KeyPress-L>", (1, "rot", "slow", 1))
        self._bind_key("<Control-Alt-Shift-KeyPress-colon>", (1, "rot", "fast", 1))
        self._bind_key("<Control-Alt-Shift-KeyPress-M>", (2, "rot", "fast", -1))
        self._bind_key("<Control-Alt-Shift-KeyPress-less>", (2, "rot", "slow", -1))
        self._bind_key("<Control-Alt-Shift-KeyPress-greater>", (2, "rot", "slow", 1))
        self._bind_key("<Control-Alt-Shift-KeyPress-question>", (2, "rot", "fast", 1))

    def _bind_key(self, key, params):
        self.root.bind(key, lambda e: self._move_object(*params))

    def _run_sim(self, *args):
        raise NotImplementedError

    def _stop_sim(self, *args):
        raise NotImplementedError

    def _snapshot(self, cmd, label=True):
        raise NotImplementedError

    def _move_object(self, axis, mode, speed, direction):
        raise NotImplementedError

    def mainloop(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = Application()
    app.mainloop()
