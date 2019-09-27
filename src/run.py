from sim.world import World
from sim.robot_arm import RobotArm

from sim.scene_tossing import SceneTossing

def main():
    # Create world
    world = World()
    scene = SceneTossing(world)

    # Spawn robot
    robot = RobotArm(world)
    robot.reset()

    robot.to_start()

    world.step_seconds(2)


    world.step_seconds(50)

    world.close()

if __name__ == "__main__":
    main()
