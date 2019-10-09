from sim.world import World
from sim.robot_arm import RobotArm

from sim.scene_tossing import SceneTossing
from sim.scene_planning_1 import ScenePlanning1

from skills.navigate import SkillNavigation
from skills.grasping import SkillGrasping

def main():
    # Create world
    world = World(gui_=False, sleep_=False)
    scene = ScenePlanning1(world)

    # Spawn robot
    robot = RobotArm(world)
    robot.reset()

    # Set up skills
    sk_nav = SkillNavigation(scene, robot._model.uid)
    sk_grasp = SkillGrasping(scene, robot)

    # -----------------------------------
    
    robot.to_start()

    world.step_seconds(1)

    sk_grasp.grasp_object(2)

    world.step_seconds(1)

    # Robot velocity control
    robot.update_velocity([0.1, 0.0, 0.0], 0.1)

    world.step_seconds(5)

    # robot.transition_cartesian(robot.start_pos, robot.start_orient)

    # Run move skill
    # res = sk_nav.move_to_object(2)
    # print("Move result: " + str(res))

    world.step_seconds(50)

    world.close()

if __name__ == "__main__":
    main()
