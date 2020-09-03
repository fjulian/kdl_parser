import unittest
import os
import numpy as np
import pybullet as p

# Simulation
from highlevel_planning.sim.world import World
from highlevel_planning.sim.robot_arm import RobotArm
from highlevel_planning.sim.scene_planning_1 import ScenePlanning1

# Skills
from highlevel_planning.skills.navigate import SkillNavigate
from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.skills.placing import SkillPlacing
from highlevel_planning.skills import pddl_descriptions
from highlevel_planning.knowledge.predicates import Predicates

# Learning
from highlevel_planning.knowledge.knowledge_base import KnowledgeBase
from highlevel_planning.learning.explorer import Explorer
from highlevel_planning.learning.pddl_extender import PDDLExtender
from highlevel_planning.learning import logic_tools
from highlevel_planning.learning.precondition_discovery import precondition_discovery
from highlevel_planning.learning.sequence_completion import complete_sequence

# Other
from highlevel_planning.tools.config import ConfigYaml

BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestExplorer(unittest.TestCase):
    kb = None

    @classmethod
    def setUpClass(cls):
        cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

        cls.kb = KnowledgeBase(BASEDIR, domain_name="chimera-test")
        cls.kb.goals = []

        # Add basic skill descriptions
        skill_descriptions = pddl_descriptions.get_action_descriptions()
        for skill_name, description in skill_descriptions.items():
            cls.kb.add_action(
                action_name=skill_name, action_definition=description, overwrite=True,
            )

        # Add required types
        cls.kb.add_type("robot")
        cls.kb.add_type("navgoal")  # Anything we can navigate to
        cls.kb.add_type("position", "navgoal")  # Pure positions
        cls.kb.add_type("item", "navgoal")  # Anything we can grasp

        # Add origin
        cls.kb.add_object("origin", "position", np.array([0.0, 0.0, 0.0]))
        cls.kb.add_object("manual_position1", "position", np.array([2.5, 0.2, 0.8]))
        cls.kb.add_object("manual_position2", "position", np.array([3.5, -0.25, 0.8]))
        cls.kb.add_object("robot1", "robot")

        # -----------------------------------

        # Create world
        world = World(style="gui", sleep_=False, load_objects=True)
        scene = ScenePlanning1(world, BASEDIR, restored_objects=None)
        robot = RobotArm(world, cfg, BASEDIR)
        robot.reset()
        robot.to_start()
        world.step_seconds(0.5)

        # -----------------------------------

        # Set up predicates
        preds = Predicates(scene, robot, cls.kb, cfg)
        cls.kb.set_predicate_funcs(preds)

        for descr in preds.descriptions.items():
            cls.kb.add_predicate(
                predicate_name=descr[0], predicate_definition=descr[1], overwrite=True
            )

        # Planning problem
        cls.kb.populate_visible_objects(scene)
        cls.kb.check_predicates()

        sk_grasp = SkillGrasping(scene, robot, cfg)
        sk_place = SkillPlacing(scene, robot)
        sk_nav = SkillNavigate(scene, robot)
        skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

        pddl_ex = PDDLExtender(cls.kb, preds)

        cls.xplorer = Explorer(skill_set, robot, scene.objects, pddl_ex, cls.kb, cfg)
        cls.xplorer.current_state_id = p.saveState()

        # Save the state the robot is currently in
        cls.default_state_id = p.saveState()

    def test_sequence_completion(self):
        sequence = ["grasp", "place"]
        parameters = [
            {"obj": "lid1", "rob": "robot1"},
            {"obj": "cube1", "pos": "origin", "rob": "robot1"},
        ]
        completion_result = complete_sequence(sequence, parameters, self.kb)
        self.assertEqual(
            completion_result[0],
            [
                "grasp",
                "nav-in-reach",
                "place",
                "nav-in-reach",
                "grasp",
                "nav-in-reach",
                "place",
            ],
        )
        self.assertEqual(
            completion_result[1],
            [
                {"rob": "robot1", "obj": "lid1"},
                {"goal_pos": "origin", "rob": "robot1", "current_pos": "lid1"},
                {"rob": "robot1", "obj": "lid1", "pos": "origin"},
                {"goal_pos": "cube1", "rob": "robot1", "current_pos": "origin"},
                {"rob": "robot1", "obj": "cube1"},
                {"goal_pos": "origin", "rob": "robot1", "current_pos": "cube1"},
                {"rob": "robot1", "obj": "cube1", "pos": "origin"},
            ],
        )
        self.assertEqual(completion_result[2], ["nav-in-reach"])
        self.assertEqual(
            completion_result[3],
            [{"goal_pos": "lid1", "rob": "robot1", "current_pos": "origin"}],
        )

    def test_single_action_completion(self):
        sequence = ["grasp"]
        parameters = [{"obj": "lid1", "rob": "robot1"}]
        completion_result = complete_sequence(sequence, parameters, self.kb)
        (
            completed_sequence,
            completed_parameters,
            precondition_sequence,
            precondition_parameters,
            _,
        ) = completion_result
        sequence_preconds = logic_tools.determine_sequence_preconds(
            self.kb, sequence, parameters
        )
        precondition_plan = self.kb.solve_temp(sequence_preconds)
        if not precondition_plan:
            self.assertTrue(False)
        precondition_sequence2, precondition_parameters2 = precondition_plan
        self.assertEqual(completed_sequence, sequence)
        self.assertEqual(completed_parameters, parameters)
        self.assertEqual(precondition_sequence, precondition_sequence2)
        self.assertEqual(precondition_parameters, precondition_parameters2)

    def test_precondition_discovery(self):
        sequence = ["place", "place"]
        parameters = [
            {"obj": "lid1", "pos": "manual_position1", "rob": "robot1"},
            {"obj": "cube1", "pos": "manual_position2", "rob": "robot1"},
        ]
        completion_result = complete_sequence(sequence, parameters, self.kb)

        goal_objects = ["cube1", "container1"]
        closeby_objects = self.xplorer._get_items_closeby(goal_objects, 0.5)
        ret = precondition_discovery(
            goal_objects + closeby_objects, completion_result, self.xplorer
        )
        self.assertIn(("on", ("container1", "lid1")), ret)

    def test_precondition_position_sampling(self):
        """
        Test whether we solved the problem from hlp_logbook 31.08.2020.
        Position to place the lid before grasping the cube should be sampled.
        """
        sequence = ["grasp", "place"]
        parameters = [
            {"obj": "lid1", "rob": "robot1"},
            {"obj": "cube1", "pos": "origin", "rob": "robot1"},
        ]
        # TODO implement this and the code that this will test.

    @classmethod
    def tearDownClass(cls):
        for filename in os.listdir(os.path.join(cls.kb.knowledge_dir, "main")):
            os.remove(os.path.join(cls.kb.knowledge_dir, "main", filename))
        for filename in os.listdir(os.path.join(cls.kb.knowledge_dir, "explore")):
            os.remove(os.path.join(cls.kb.knowledge_dir, "explore", filename))


if __name__ == "__main__":
    unittest.main()
