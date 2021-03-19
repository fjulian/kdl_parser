import numpy as np
import os
import atexit
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

# Other
from highlevel_planning_py.tools.config import ConfigYaml
from highlevel_planning_py.tools import run_util
from highlevel_planning_py.tools.reporter import Reporter

from highlevel_planning_py.exploration.exploration_tools import get_items_closeby

# ----------------------------------------------------------------------

DATADIR = os.path.join(os.path.expanduser("~"), "Data", "highlevel_planning")
SRCROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETDIR = os.path.join(SRCROOT, "data", "models")


def exit_handler(rep: Reporter):
    rep.write_result_file()


def print_plan(sequence, parameters):
    print("---------------------------------------------------")
    print("Found plan:")
    for idx, seq_item in enumerate(sequence):
        print(f"{seq_item} {parameters[idx]}")
    print("---------------------------------------------------")


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
    savedir = os.path.join(DATADIR, "simulator")
    objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

    # Load config file
    cfg = ConfigYaml(os.path.join(SRCROOT, "config", "mcts.yaml"))

    time_now = datetime.now()
    time_string = time_now.strftime("%y%m%d-%H%M%S")
    # rep = Reporter(DATADIR, cfg, time_string, args.noninteractive)
    # atexit.register(exit_handler, rep)

    # Populate simulation
    scene, world = run_util.setup_pybullet_world(
        ScenePlanning1, ASSETDIR, args, savedir, objects
    )
    robot = run_util.setup_robot(world, cfg, ASSETDIR, robot_mdl)

    # Save state
    run_util.save_pybullet_sim(args, savedir, scene, robot)

    # -----------------------------------

    kb, preds = run_util.setup_knowledge_base(DATADIR, scene, robot, cfg, time_string)

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot, cfg)
    sk_place = SkillPlacing(scene, robot)
    sk_nav = SkillNavigate(scene, robot)
    skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

    # PDDL extender
    pddl_ex = PDDLExtender(kb, preds)

    # Set up exploration
    xplorer = Explorer(skill_set, robot, scene.objects, pddl_ex, kb, cfg)
    goal_objects = xplorer._get_items_goal()
    closeby_objects = get_items_closeby(
        goal_objects,
        scene.objects,
        robot.model.uid,
        distance_limit=0.4,  # TODO move magic number to parameters
    )
    relevant_objects = goal_objects + closeby_objects

    # Set up MCTS
    graph = nx.DiGraph()
    mcts_state = mcts.HLPState(True, 0, world.client_id, xplorer)
    mcts_root_node = mcts.HLPTreeNode(
        mcts_state, xplorer, graph, relevant_objects=relevant_objects
    )
    mcts_search = mcts.HLPTreeSearch(mcts_root_node)

    # ---------------------------------------------------------------

    mcts_search.tree_search()

    save = True
    if save:
        figure, ax = plt.subplots()
        mcts_search.figure = figure
        mcts_search.ax = ax
        mcts_search.plot_graph(mcts_root_node)
        plt.show()


if __name__ == "__main__":
    main()
