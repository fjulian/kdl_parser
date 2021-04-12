import numpy as np
import os
import json
import atexit
import pickle
from datetime import datetime
import networkx as nx
import matplotlib as mpl
import matplotlib.pyplot as plt

mpl.use("TkAgg")

# Simulation
from highlevel_planning_py.sim.scene_planning_1 import ScenePlanning1

# Skills
from highlevel_planning_py.skills.navigate import SkillNavigate
from highlevel_planning_py.skills.grasping import SkillGrasping
from highlevel_planning_py.skills.placing import SkillPlacing

# Learning
from highlevel_planning_py.exploration.explorer import Explorer
from highlevel_planning_py.exploration.pddl_extender import PDDLExtender
from highlevel_planning_py.exploration import mcts
from highlevel_planning_py.exploration.logic_tools import determine_relevant_predicates

# Other
from highlevel_planning_py.tools.config import ConfigYaml
from highlevel_planning_py.tools import run_util
from highlevel_planning_py.tools.reporter import Reporter

from highlevel_planning_py.exploration.exploration_tools import get_items_closeby

# ----------------------------------------------------------------------

SRCROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATHS = {
    "data_dir": os.path.join(os.path.expanduser("~"), "Data", "highlevel_planning"),
    "src_root_dir": SRCROOT,
    "asset_dir": os.path.join(SRCROOT, "data", "models"),
    "bin_dir": os.path.join(SRCROOT, "bin"),
}


def mcts_exit_handler(node, time_string, config, metrics):
    savedir = os.path.join(PATHS["data_dir"], "mcts")
    os.makedirs(savedir, exist_ok=True)

    figure, ax = plt.subplots()
    mcts.plot_graph(node.graph, node, figure, ax, explorer=None)
    filename = "{}_mcts_tree.png".format(time_string)
    figure.savefig(os.path.join(savedir, filename))

    data = dict()
    data["tree"] = node
    data["config"] = config._cfg
    data["metrics"] = metrics

    filename = "{}_data.pkl".format(time_string)
    with open(os.path.join(savedir, filename), "wb") as f:
        pickle.dump(data, f)

    filename = "{}_metrics.txt".format(time_string)
    with open(os.path.join(savedir, filename), "w") as f:
        for key, value in metrics.items():
            f.write(f"{key:42}: {value}\n")

    filename = "{}_config.txt".format(time_string)
    with open(os.path.join(savedir, filename), "w") as f:
        json.dump(config._cfg, f)

    print(f"Saved everything. Time string: {time_string}")


def main():
    # Command line arguments
    args = run_util.parse_arguments()

    # Seed RNGs
    if not args.no_seed:
        print("Seeding RNG")
        np.random.seed(0)

    if args.method == "direct" and args.reuse_objects:
        raise RuntimeError("Cannot reload objects when in direct mode.")

    # Load existing simulation data if desired
    savedir = os.path.join(PATHS["data_dir"], "simulator")
    objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

    # Load config file
    cfg = ConfigYaml(os.path.join(SRCROOT, "config", "mcts.yaml"))

    time_now = datetime.now()
    time_string = time_now.strftime("%y%m%d-%H%M%S")
    # rep = Reporter(DATADIR, cfg, time_string, args.noninteractive)
    # atexit.register(exit_handler, rep)

    # Populate simulation
    scene, world = run_util.setup_pybullet_world(
        ScenePlanning1, PATHS["asset_dir"], args, savedir, objects
    )
    robot = run_util.setup_robot(world, cfg, PATHS["asset_dir"], robot_mdl)

    # Save state
    run_util.save_pybullet_sim(args, savedir, scene, robot)

    # -----------------------------------

    kb, preds = run_util.setup_knowledge_base(PATHS, scene, robot, cfg, time_string)

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot, cfg)
    sk_place = SkillPlacing(scene, robot)
    sk_nav = SkillNavigate(scene, robot)
    skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

    # PDDL extender
    pddl_ex = PDDLExtender(kb, preds)

    # Set up exploration
    xplorer = Explorer(skill_set, robot, scene.objects, pddl_ex, kb, cfg, world)
    goal_objects = xplorer._get_items_goal()
    closeby_objects = get_items_closeby(
        goal_objects,
        scene.objects,
        robot.model.uid,
        distance_limit=0.1,  # TODO move magic number to parameters
    )
    relevant_objects = goal_objects + closeby_objects
    action_list = [act for act in kb.actions if act not in kb.meta_actions]
    relevant_predicates = determine_relevant_predicates(relevant_objects, kb)

    # Set up MCTS
    graph = nx.DiGraph()
    max_depth = cfg.getparam(["mcts", "max_depth"], default_value=10)
    mcts_state = mcts.HLPState(
        True, 0, world.client_id, xplorer, relevant_predicates, max_depth
    )
    mcts_root_node = mcts.HLPTreeNode(
        mcts_state, action_list, graph, relevant_objects=relevant_objects
    )
    mcts_search = mcts.HLPTreeSearch(mcts_root_node, xplorer, cfg)
    # atexit.register(mcts_exit_handler, node=mcts_root_node, time_string=time_string)

    # ---------------------------------------------------------------

    metrics = mcts_search.tree_search()

    mcts_exit_handler(mcts_root_node, time_string, cfg, metrics)


if __name__ == "__main__":
    main()
