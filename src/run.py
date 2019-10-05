from sim.world import World
from sim.robot_arm import RobotArm

from sim.scene_tossing import SceneTossing
from sim.scene_planning_1 import ScenePlanning1

from skills.navigate import SkillNavigation

def main():
    # Create world
    world = World(gui_=True, sleep_=False)
    scene = ScenePlanning1(world)

    # Spawn robot
    robot = RobotArm(world)
    robot.reset()

    # Set up skills
    sk_nav = SkillNavigation(scene, robot._model.uid)

    # -----------------------------------
    
    robot.to_start()

    world.step_seconds(1)

    # robot.update_velocity([0.1, 0.0, 0.0], 0.1)
    res = sk_nav.move_to_object(2)
    print("Move result: " + str(res))

    world.step_seconds(50)

    world.close()

if __name__ == "__main__":
    main()
