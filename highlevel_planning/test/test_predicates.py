import unittest
import os
import numpy as np
import pybullet as p

# Simulation
from highlevel_planning.sim.world import World
from highlevel_planning.sim.robot_arm import RobotArm
from support.scene_test_1 import SceneTest1

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


class TestPredicates(unittest.TestCase):
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
                action_name=skill_name, action_definition=description, overwrite=True
            )

        # Add required types
        cls.kb.add_type("robot")
        cls.kb.add_type("navgoal")  # Anything we can navigate to
        cls.kb.add_type("position", "navgoal")  # Pure positions
        cls.kb.add_type("item", "navgoal")  # Anything we can grasp

        # Add origin
        cls.kb.add_object("origin", "position", np.array([0.0, 0.0, 0.0]))
        cls.kb.add_object("robot1", "robot")

        # -----------------------------------

        # Create world
        world = World(style="gui", sleep_=False, load_objects=True)
        scene = SceneTest1(world, BASEDIR, restored_objects=None)
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

    def test_inside(self):
        self.assertFalse(self.kb.predicate_funcs.call["inside"]("container1", "cube1"))
        self.assertTrue(self.kb.predicate_funcs.call["inside"]("container1", "cube2"))
        self.assertFalse(self.kb.predicate_funcs.call["inside"]("cube2", "container1"))

    @classmethod
    def tearDownClass(cls):
        for filename in os.listdir(os.path.join(cls.kb.knowledge_dir, "main")):
            os.remove(os.path.join(cls.kb.knowledge_dir, "main", filename))
        for filename in os.listdir(os.path.join(cls.kb.knowledge_dir, "explore")):
            os.remove(os.path.join(cls.kb.knowledge_dir, "explore", filename))


if __name__ == "__main__":
    unittest.main()
