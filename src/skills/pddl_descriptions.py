from grasping import get_grasping_description
from navigate import get_nav_description

def get_action_description(action):
    if action == "grasp":
        return get_grasping_description()
    elif action == "nav":
        return get_nav_description()
    else:
        raise ValueError("Description of non-existent skill requested.")
