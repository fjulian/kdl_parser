import pybullet as pb
from time import time
import numpy as np
from collections import defaultdict
from itertools import product
import networkx as nx
import matplotlib.pyplot as plt


from highlevel_planning_py.execution.es_sequential_execution import (
    execute_plan_sequentially,
)
from highlevel_planning_py.exploration.logic_tools import (
    find_all_parameter_assignments,
    parametrize_predicate,
    measure_predicates,
)


MAX_DEPTH = 10
DEBUG = True


def plot_graph(graph, current_node, fig, ax, explorer):
    pos = nx.drawing.nx_pydot.graphviz_layout(graph, prog="dot")
    if len(pos) == 0:
        return
    ax.clear()

    labels = dict()
    for node in graph.nodes._nodes:
        color = "#1f78b4"  # blue
        if node is current_node:
            color = "#ff0000"  # red
        elif node.own_action is not None:
            if (
                node.own_action[0][0] == "grasp"
                and node.own_action[1][0]["obj"] == "tall_box"
            ):
                color = "#eaff80"  # light green
            elif (
                "nav" in node.own_action[0][0]
                and node.own_action[1][0]["goal_pos"] == "shelf"
            ):
                color = "#00b300"  # dark green

        if node.own_action is not None:
            labels[node] = node.own_action[0][0][0]

        marker = "o"
        if node.is_terminal(explorer):
            if node.state.goal_reached(explorer):
                marker = "*"
                color = "#ff00ff"  # pink
            else:
                marker = "^"

        nx.draw_networkx_nodes(
            graph, pos, nodelist=[node], node_color=color, node_shape=marker, ax=ax
        )

    nx.draw_networkx_labels(graph, pos, labels, font_size=13)
    nx.draw_networkx_edges(graph, pos, ax=ax)

    fig.canvas.draw()
    fig.canvas.flush_events()


class HLPTreeSearch:
    def __init__(self, root, explorer, config):
        self.root = root

        self.exp = explorer

        self.time_budget = config.getparam(["mcts", "search_budget_sec"])

        # if DEBUG:
        plt.ion()
        self.figure, self.ax = plt.subplots(figsize=(25, 18))

    def tree_search(self):
        start_time = time()
        counter = 0
        counter2 = 0
        while time() - start_time < self.time_budget:
            counter += 1
            current_node = self.root
            while not current_node.is_terminal(self.exp):
                counter2 += 1
                if DEBUG:  # and counter2 % 10 == 0:
                    plot_graph(
                        self.root.graph, current_node, self.figure, self.ax, self.exp
                    )
                if current_node.check_expanding(self.exp):
                    # Expand
                    current_node = current_node.expand(self.exp)
                    # break
                else:
                    # Select a child to continue from
                    current_node = current_node.select_child(self.exp)

            # result = current_node.rollout()
            result = current_node.state.game_result(self.exp)
            result = 0 if result is None else result
            if result == 1:
                plot_graph(self.root.graph, self.root, self.figure, self.ax, self.exp)
                bla = input("fadsf")
            current_node.backpropagate(result)
            # print("----------------------------------------------")
            # self.root.print()
            print(
                "Iteration {}. Current successes: {}".format(
                    counter, self.root.results[1]
                )
            )


class HLPTreeNode:
    def __init__(
        self,
        state,
        action_list,
        graph,
        relevant_objects=None,
        parent=None,
        own_action=None,
    ):
        self.state = state
        self.graph = graph
        self.parent = parent
        self.relevant_objects = relevant_objects
        self.own_action = own_action

        self.action_list = action_list
        self.children = list()
        self.child_actions = list()

        self.num_visited = 0
        self.cannot_expand = False
        self.overwrite_terminal = False
        self.results = defaultdict(int)

        if self.parent is not None:
            graph.add_edge(self.parent, self)

    def is_terminal(self, explorer):
        if self.overwrite_terminal:
            return True
        return self.state.is_game_over(explorer)

    def check_expanding(self, explorer):
        if self.cannot_expand:
            return False

        # If there is no non-terminal child, expand
        all_terminal = True
        for child in self.children:
            all_terminal &= child.is_terminal(explorer)
        if all_terminal:
            return True

        alpha = 0.6
        return (
            True
            if self.num_visited == 0 or len(self.children) == 0
            else np.floor(self.num_visited ** alpha)
            > np.floor((self.num_visited - 1) ** alpha)
        )

    def expand(self, explorer):
        max_tries = 50
        counter = 0
        while counter < max_tries:
            counter += 1
            sequence_tuple = self._sample_feasible_step(explorer)
            if sequence_tuple is False:
                self.cannot_expand = True
                return self
            if sequence_tuple in self.child_actions:
                continue
            new_state = self.state.move(sequence_tuple, explorer)
            new_child = HLPTreeNode(
                new_state,
                self.action_list,
                self.graph,
                relevant_objects=self.relevant_objects,
                parent=self,
                own_action=sequence_tuple,
            )
            self.children.append(new_child)
            self.child_actions.append(sequence_tuple)
            return new_child
        return None

    def _sample_step(self, explorer):
        sequence = explorer._sample_sequence(length=1)
        parameters, _ = explorer._sample_parameters(
            sequence, relevant_objects=self.relevant_objects
        )
        sequence_tuple = (tuple(sequence), tuple(parameters))
        return sequence_tuple

    def _sample_feasible_step(self, explorer):
        self.state.restore_state(explorer)

        feasible_moves = list()
        feasible_navgoals = set()
        for action in self.action_list:
            # Skip nav action if the last action was already a nav action
            if self.own_action is not None:
                if "nav" in self.own_action[0][0] and "nav" in action:
                    continue

            # Get parameters
            parameters = explorer.knowledge_base.actions[action]["params"]
            parameter_assignments = find_all_parameter_assignments(
                parameters,
                self.relevant_objects + ["origin", "grasp0", "grasp1"],
                explorer.knowledge_base,
            )

            # Sample positions
            num_position_samples = 2
            num_reachable_position_samples = 2
            max_reachable_tries = 60
            for i, parameter in enumerate(parameters):
                if explorer.knowledge_base.type_x_child_of_y(parameter[1], "position"):
                    for j in range(num_position_samples):
                        position = explorer.sample_position(self.relevant_objects)
                        position_name = explorer.knowledge_base.add_temp_object(
                            object_type="position", object_value=position
                        )
                        parameter_assignments[i].append(position_name)
                    k = 0
                    k_count = 0
                    while k < num_reachable_position_samples:
                        k_count += 1
                        position = explorer.sample_position(self.relevant_objects)
                        position_name = explorer.knowledge_base.add_temp_object(
                            object_type="position", object_value=position
                        )
                        if explorer.knowledge_base.predicate_funcs.call["in-reach"](
                            position_name, None
                        ):
                            parameter_assignments[i].append(position_name)
                            k += 1
                        else:
                            explorer.knowledge_base.remove_temp_object(position_name)
                        if k_count > max_reachable_tries:
                            break

            # For each parameterization, get all possible assignments
            parameter_dicts = list()
            for parametrization in product(*parameter_assignments):
                parameter_dict = {
                    parameters[i][0]: parametrization[i] for i in range(len(parameters))
                }
                parameter_dicts.append(parameter_dict)

            # Check which of them are feasible at the current state
            preconditions = explorer.knowledge_base.actions[action]["preconds"]
            for parameter_dict in parameter_dicts:
                if "nav" in action:
                    if (
                        parameter_dict["goal_pos"] in feasible_navgoals
                        or parameter_dict["goal_pos"] == parameter_dict["current_pos"]
                    ):
                        continue

                feasible = True
                for precond in preconditions:
                    parameterized_precond = parametrize_predicate(
                        precond, parameter_dict
                    )
                    try:
                        idx = self.state.predicate_specs.index(
                            (parameterized_precond[0], parameterized_precond[2])
                        )
                        res = self.state.measured_predicates[idx]
                    except ValueError:
                        res = explorer.knowledge_base.predicate_funcs.call[precond[0]](
                            *parameterized_precond[2]
                        )
                    feasible &= res == precond[1]
                    if not feasible:
                        break
                sequence_tuple = ((action,), (parameter_dict,))
                if feasible and sequence_tuple not in self.child_actions:
                    if "nav" in action:
                        feasible_navgoals.add(parameter_dict["goal_pos"])
                    feasible_moves.append(sequence_tuple)

        if len(feasible_moves) == 0:
            return False
        selected_action = np.random.randint(len(feasible_moves))
        return feasible_moves[selected_action]

    def rollout(self, explorer):
        current_rollout_state = self.state
        while not current_rollout_state.is_game_over():
            max_tries = 50
            counter = 0
            new_state = None
            while counter < max_tries:
                counter += 1
                sequence_tuple = self._sample_step(explorer)
                new_state = current_rollout_state.move(sequence_tuple)
                if new_state.success:
                    break
            current_rollout_state = new_state
        return current_rollout_state.game_result()

    def select_child(self, explorer, exploration_constant=np.sqrt(2)):
        scores = list()
        for child in self.children:
            if not child.is_terminal(explorer):
                avg_result = float(child.results[1]) / float(child.num_visited)
                score = avg_result + exploration_constant * np.sqrt(
                    np.log(self.num_visited) / float(child.num_visited)
                )
            else:
                score = -np.inf
            scores.append(score)
        if len(scores) == 0:
            if self.cannot_expand:
                self.overwrite_terminal = True
            return self
        max_idx = np.argmax(scores)
        return self.children[max_idx]

    def backpropagate(self, result):
        self.num_visited += 1
        self.results[result] += 1
        if self.parent:
            self.parent.backpropagate(result)

    def print(self):
        spacer = "-" * (self.state._depth - 1)
        spacer += "|"
        print(
            spacer
            + " ({}) {} Terminal: {}".format(
                self.num_visited, self.state.action_str, self.is_terminal(explorer=None)
            )
        )
        for child in self.children:
            child.print()


class HLPState:
    def __init__(
        self, success, depth, pb_client_id, explorer, predicate_specs, max_depth
    ):
        self._depth = depth
        self._max_depth = max_depth

        # Save state
        self._bullet_state = explorer.world.save_state()
        self._bullet_client_id = pb_client_id
        self.arm_state = explorer.robot.desired_arm
        self.finger_state = explorer.robot.desired_fingers

        # Evaluate predicates
        self.predicate_specs = predicate_specs
        self.measured_predicates = measure_predicates(
            predicate_specs, explorer.knowledge_base
        )

        self.success = success
        self.goal_reached_cache = None

    def game_result(self, explorer):
        if not self.success:
            return 0
        elif self._depth >= self._max_depth:
            return 0
        elif self.goal_reached(explorer):
            return 1
        else:
            return None

    def is_game_over(self, explorer):
        return self.game_result(explorer) is not None

    def move(self, action, explorer):
        self.restore_state(explorer)
        success = execute_plan_sequentially(
            action[0], action[1], explorer.skill_set, explorer.knowledge_base
        )
        new_state = HLPState(
            success,
            self._depth + 1,
            self._bullet_client_id,
            explorer,
            self.predicate_specs,
            self._max_depth,
        )
        return new_state

    def restore_state(self, explorer):
        explorer.world.restore_state(self._bullet_state)
        explorer.robot.set_joints(self.arm_state)
        explorer.robot.set_fingers(self.finger_state)

    def goal_reached(self, explorer):
        if self.goal_reached_cache is None:
            if explorer is None:
                return False
            self.restore_state(explorer)
            self.goal_reached_cache = explorer.knowledge_base.test_goals()
        return self.goal_reached_cache
