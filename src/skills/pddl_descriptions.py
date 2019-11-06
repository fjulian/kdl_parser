from grasping import get_grasping_description

def get_action_description(action):
    if action == "grasp":
        return get_grasping_description()
    else:
        raise ValueError("Description of non-existent skill requested.")