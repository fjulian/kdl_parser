from sim.world import World
from sim.robot_arm import RobotArm


def main():

    # Create world
    world = World()
    world.add_plane()
    robot = RobotArm(world)
    robot.reset()

    robot.to_start()

    world.step_seconds(2)

    curr_joints = robot.get_joints()
    print("Curr joints: ", curr_joints)
    print("Start joints: ", robot.start_cmd)

    world.step_seconds(50)

    world.close()

if __name__ == "__main__":
    main()
