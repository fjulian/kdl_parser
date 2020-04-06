from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.tools.util import get_combined_aabb
import pybullet as p
import numpy as np


class FakePredicates:
    def __init__(self):
        self.descriptions = {
            "carry": [["rov", "rover"], ["sam", "sample"]],
            "is-dropping-dock": [["wp", "waypoint"]],
        }
