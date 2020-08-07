import unittest
import os
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

# Other
from highlevel_planning.tools.config import ConfigYaml

BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestExplorer(unittest.TestCase):
    def setUp(self):
        cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

        self.kb = KnowledgeBase(BASEDIR, domain_name="chimera-test")
        self.kb.goals = []

        # Add basic skill descriptions
        skill_descriptions = pddl_descriptions.get_action_descriptions()
        for skill_name, description in skill_descriptions.items():
            self.kb.add_action(
                action_name=skill_name, action_definition=description, overwrite=True,
            )

        # Add required types
        self.kb.add_type("robot")
        self.kb.add_type("navgoal")  # Anything we can navigate to
        self.kb.add_type("position", "navgoal")  # Pure positions
        self.kb.add_type("item", "navgoal")  # Anything we can grasp

        # Add origin
        self.kb.add_object("origin", "position", np.array([0.0, 0.0, 0.0]))
        self.kb.add_object("robot1", "robot")

        # -----------------------------------

        # Create world
        world = World(gui=False, sleep_=False, load_objects=True)
        scene = ScenePlanning1(world, BASEDIR, restored_objects=None)
        robot = RobotArm(world, cfg, BASEDIR)
        robot.reset()
        robot.to_start()
        world.step_seconds(0.5)

        # -----------------------------------

        # Set up predicates
        preds = Predicates(scene, robot, self.kb, cfg)
        self.kb.set_predicate_funcs(preds)

        for descr in preds.descriptions.items():
            self.kb.add_predicate(
                predicate_name=descr[0], predicate_definition=descr[1], overwrite=True
            )

        # Planning problem
        self.kb.populate_visible_objects(scene)
        self.kb.check_predicates()

        sk_grasp = SkillGrasping(scene, robot, cfg)
        sk_place = SkillPlacing(scene, robot)
        sk_nav = SkillNavigate(scene, robot)
        skill_set = {"grasp": sk_grasp, "nav": sk_nav, "place": sk_place}

        pddl_ex = PDDLExtender(self.kb, preds)

        self.xplorer = Explorer(skill_set, robot, scene.objects, pddl_ex, self.kb, cfg)

    def test_sequence_completion(self):
        sequence = ["grasp", "place"]
        parameters = [
            {"obj": "lid1", "rob": "robot1"},
            {"obj": "cube1", "pos": "origin", "rob": "robot1"},
        ]
        self.xplorer.complete_sequence(sequence, parameters)
        self.assertEqual(1, 1)

    def tearDown(self):
        for filename in os.listdir(os.path.join(self.kb.knowledge_dir, "main")):
            os.remove(os.path.join(self.kb.knowledge_dir, "main", filename))
        for filename in os.listdir(os.path.join(self.kb.knowledge_dir, "explore")):
            os.remove(os.path.join(self.kb.knowledge_dir, "explore", filename))


if __name__ == "__main__":
    unittest.main()
