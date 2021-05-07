import os
import pickle
from collections import OrderedDict
from datetime import datetime
from copy import deepcopy

from highlevel_planning_py.knowledge.knowledge_base import KnowledgeBase
from highlevel_planning_py.tools.config import ConfigYaml


class Reporter:
    def __init__(
        self,
        paths,
        config: ConfigYaml,
        domain_name,
        time_string: str = None,
        domain_file="_domain.pkl",
    ):
        self.paths = paths
        self.domain_name = domain_name
        self.domain_file = domain_file
        self.data = dict()
        self.metrics = OrderedDict()
        if time_string is None:
            time_stamp = datetime.now()
            self.metrics["time"] = time_stamp.strftime("%y%m%d-%H%M%S")
        else:
            self.metrics["time"] = time_string
        self.metrics["domain_file"] = domain_file
        print(f"Time string for reporting is {self.metrics['time']}")
        self.data["configuration"] = deepcopy(config._cfg)

        self.planning_idx = 0
        self.explore_idx = 0
        self.run_idx = 0

    def _extract_kb_metrics(self, kb, prefix: str):
        self.metrics[f"{prefix}_actions"] = str(len(kb.actions))
        self.metrics[f"{prefix}_types"] = str(len(kb.types))
        self.metrics[f"{prefix}_objects"] = str(len(kb.objects))
        self.metrics[f"{prefix}_visible_objects"] = str(len(kb.visible_objects))
        self.metrics[f"{prefix}_object_predicates"] = str(len(kb.object_predicates))
        self.metrics[f"{prefix}_initial_state_predicates"] = str(
            len(kb.initial_state_predicates)
        )
        self.metrics[f"{prefix}_lookup_table"] = str(len(kb.lookup_table))
        self.metrics[f"{prefix}_parameterizations"] = str(len(kb.parameterizations))
        self.metrics[f"{prefix}_meta_actions"] = str(len(kb.meta_actions))

    def report_after_planning(self, plan, kb):
        self.data[f"plan_{self.planning_idx}_plan"] = deepcopy(plan)
        success = True if plan is not False else False
        self.metrics[f"plan_{self.planning_idx}_success"] = str(success)
        kb.save_domain(filename_appendix=f"_plan_step_{self.planning_idx}")
        self.planning_idx += 1

    def report_after_execution(self, res: bool):
        self.metrics[f"exec_{self.run_idx}_success"] = str(res)
        self.run_idx += 1

    def report_before_exploration(self, knowledge_base: KnowledgeBase, plan):
        kb_clone = KnowledgeBase(
            self.paths, domain_name=self.domain_name, domain_file=self.domain_file
        )
        kb_clone.duplicate(knowledge_base)
        self.data[f"explore_{self.explore_idx}_kb_before"] = kb_clone
        self.metrics[f"explore_{self.explore_idx}_goal"] = str(kb_clone.goals)
        self._extract_kb_metrics(kb_clone, f"explore_{self.explore_idx}_kb_before")
        self.data[f"explore_{self.explore_idx}_plan_before"] = deepcopy(plan)
        self.metrics[f"explore_{self.explore_idx}_plan_before_success"] = str(
            True if plan is not False else False
        )

    def report_after_exploration(
        self, knowledge_base: KnowledgeBase, exploration_metrics: OrderedDict
    ):
        kb_clone = KnowledgeBase(
            self.paths, domain_name=self.domain_name, domain_file=self.domain_file
        )
        kb_clone.duplicate(knowledge_base)
        self.data[f"explore_{self.explore_idx}_kb_after"] = kb_clone
        self._extract_kb_metrics(kb_clone, f"explore_{self.explore_idx}_kb_after")
        for key, value in exploration_metrics.items():
            self.metrics[f"explore_{self.explore_idx}_{key}"] = value
        self.explore_idx += 1

    def write_result_file(self):
        savedir = os.path.join(self.paths["data_dir"], "reports")
        if not os.path.isdir(savedir):
            os.makedirs(savedir)

        # Write index file
        savefile = os.path.join(savedir, f"{self.metrics['time']}_index.txt")
        with open(savefile, "w") as f:
            for key, value in self.metrics.items():
                f.write(f"{key:42}: {value}\n")

        # Write data
        self.data["metrics"] = self.metrics
        with open(os.path.join(savedir, f"{self.metrics['time']}_data.pkl"), "wb") as f:
            pickle.dump(self.data, f)

        print(f"Reporter wrote result files. Time string: {self.metrics['time']}.")
