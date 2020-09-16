import os
import pickle
from collections import OrderedDict
from datetime import datetime
from copy import deepcopy

from highlevel_planning.knowledge.knowledge_base import KnowledgeBase


class Reporter:
    def __init__(self, basedir: str):
        self.basedir = basedir
        self.data = dict()
        self.metrics = OrderedDict()
        self.time_stamp = datetime.now()
        self.metrics["time"] = self.time_stamp.strftime("%Y-%m-%d_%H-%M-%S")
        self.metrics["description"] = input("Experiment description: ")

    def report_before_exploration(self, knowledge_base: KnowledgeBase, plan):
        kb_clone = KnowledgeBase(self.basedir)
        kb_clone.duplicate(knowledge_base)
        self.data["kb_before"] = kb_clone
        self.data["plan_before"] = deepcopy(plan)
        self.metrics["plan_before_success"] = True if plan is not False else False

    def report_after_exploration(
        self, knowledge_base: KnowledgeBase, exploration_metrics: OrderedDict
    ):
        self.data["kb_after"] = deepcopy(knowledge_base)
        self.metrics.update(exploration_metrics)

    def report_after_planning(self, plan):
        self.data["plan_after"] = deepcopy(plan)
        self.metrics["plan_after_success"] = True if plan is not False else False

    def write_result_file(self):
        savedir = os.path.join(self.basedir, "data", "reports")
        if not os.path.isdir(savedir):
            os.makedirs(savedir)

        # Write index file
        savefile = os.path.join(savedir, f"{self.time_stamp}_index.txt")
        with open(savefile, "w") as f:
            for key, value in self.metrics.items():
                f.write(f"{key:20}: {value}\n")

        # Write data
        with open(os.path.join(savedir, f"{self.time_stamp}_data.pkl"), "wb") as f:
            pickle.dump(self.data, f)
