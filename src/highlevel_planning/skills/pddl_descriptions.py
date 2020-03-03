from grasping import get_grasping_description
from navigate import get_nav_description
from placing import get_placing_description

def get_action_description(action):
    if action == "grasp":
        return get_grasping_description()
    elif action == "nav":
        return get_nav_description()
    elif action == "place":
        return get_placing_description()
    else:
        raise ValueError("Description of non-existent skill requested.")
