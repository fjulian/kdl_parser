
# Simulation
from sim.world import World
from sim.robot_arm import RobotArm
from sim.scene_planning_1 import ScenePlanning1

# Skills
from skills.navigate import SkillNavigation
# from skills.grasping import SkillGrasping
from execution.bt import ExecutionSystem
from skills import pddl_descriptions

# Interface to BT
import py_trees

# Interface to planner and PDDL
from pddl_interface import pddl_file_if, planner_interface

# -------------------------------------------------------------


def main():

    # Set up planner interface and domain representation

    pddl_if = pddl_file_if.PDDLFileInterface(domain_dir="knowledge/chimera/domain", domain_name="chimera")
    temp = pddl_descriptions.get_grasping_description()
    pddl_if.add_action(action_name=temp[0], action_definition=temp[1], overwrite=False)
    pddl_if.save_domain()

    # -----------------------------------

    # Create world
    world = World(gui_=True, sleep_=True)
    scene = ScenePlanning1(world)

    # Spawn robot
    robot = RobotArm(world)
    robot.reset()

    # -----------------------------------

    # Set up skills
    sk_nav = SkillNavigation(scene, robot._model.uid)
    # sk_grasp = SkillGrasping(scene, robot)

    # Set up behavior tree
    es = ExecutionSystem(scene, robot)

    blackboard = py_trees.blackboard.Blackboard()
    blackboard.grasp_target_name = "cube1"
    blackboard.grasp_target_link_id = None
    blackboard.grasp_target_grasp_id = None

    # -----------------------------------
    
    robot.to_start()
    # world.step_seconds(30)
    world.step_seconds(1)

    # Run move skill
    res = sk_nav.move_to_object("cube1")
    # res = sk_nav.move_to_object("cupboard")

    # Grasp the cube
    # sk_grasp.grasp_object("cube1")


    # Grasp the cupboard handle
    # sk_grasp.grasp_object("cupboard", scene.objects["cupboard"].grasp_links[3])

    # Drive back
    # robot.update_velocity([0.0, -0.1, 0.0], 0.0)
    # world.step_seconds(4)
    # robot.stop_driving()

    # Release
    # sk_grasp.release_object()
    # robot.to_start()

    # res = sk_nav.move_to_object("cube1")
    # sk_grasp.grasp_object("cube1")

    try:
        es.tree.tick_tock(sleep_ms=500, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK)
    except KeyboardInterrupt:
        es.tree.interrupt() 

    # world.step_seconds(50)

if __name__ == "__main__":
    main()
