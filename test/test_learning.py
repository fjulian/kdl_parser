import unittest
import numpy as np

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


class TestExplorer(unittest.TestCase):
    def setUp(self):

        # Set up planner interface and domain representation
        kb = KnowledgeBase(
            "test/knowledge_test/chimera", domain_name="chimera-test-domain"
        )

        kb.goals = []

        # Add basic skill descriptions
        skill_descriptions = pddl_descriptions.get_action_descriptions()
        for skill_name, description in skill_descriptions.items():
            kb.add_action(
                action_name=skill_name, action_definition=description, overwrite=True,
            )

        # Add required types
        kb.add_type("robot")
        kb.add_type("navgoal")  # Anything we can navigate to
        kb.add_type("position", "navgoal")  # Pure positions
        kb.add_type("item", "navgoal")  # Anything we can grasp

        # Add origin
        kb.add_object("origin", "position", np.array([0.0, 0.0, 0.0]))
        kb.add_object("robot1", "robot")

        # -----------------------------------

        # Create world
        world = World(gui_=False, sleep_=False, load_objects=True)
        scene = ScenePlanning1(world, restored_objects=None)
        robot = RobotArm(world)
        robot.reset()
        robot.to_start()
        world.step_seconds(0.5)

        # -----------------------------------

        # Set up predicates
        preds = Predicates(scene, robot, kb)
        kb.set_predicate_funcs(preds)

        for descr in preds.descriptions.items():
            kb.add_predicate(
                predicate_name=descr[0], predicate_definition=descr[1], overwrite=True
            )

        # Planning problem
        kb.populate_visible_objects(scene)
        kb.check_predicates()

        sk_grasp = SkillGrasping(scene, robot)
        sk_place = SkillPlacing(scene, robot)
        sk_nav = SkillNavigate(scene, robot)
        skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

        pddl_ex = PDDLExtender(kb, preds)

        self.xplorer = Explorer(skill_set, robot, scene.objects, pddl_ex, kb)

    def test_sequence_completion(self):
        sequence = ["grasp", "place"]
        parameters = [
            {"obj": "lid1", "rob": "robot1"},
            {"obj": "cube1", "pos": "origin", "rob": "robot1"},
        ]
        self.xplorer.complete_sequence(sequence, parameters)
        self.assertEqual(1, 1)


if __name__ == "__main__":
    unittest.main()
