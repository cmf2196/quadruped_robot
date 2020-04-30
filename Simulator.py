import os  # we could use this to get the path to the urdf files if you want to put them elsewhere
import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import statistics
from Robot import Robot


# ________________ Robot Starting Location and Orientation


class Simulator:

    def __init__(self):
        """
        args:
            path: String - This is the path to the robot URDF on your local computer
            pos: List - This is a list of three float values, [x , y , z]  This gives the starting position
            orientation: quaternion - this gives the robots starting orientation
                I reccomend using the getQuaternionFromEuler Pybullet function
        """

        # Set attributes

        self.robots = []  # the array of robot objects that are currently in the simulation
        self.gravity = (0, 0, -9.8)  # default gravity value
        self.current_time = 0
        self.TIME_STEP = 1

        # Start up protocol
        physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(self.gravity[0], self.gravity[1], self.gravity[2])
        planeID = p.loadURDF("plane.urdf")  # Floor
        print(planeID)

    # ____________ Methods________________________

    def load_new_robot_urdf(self, robot):
        id = p.loadURDF(robot.urdf_path, robot.get_start_pos(), robot.get_start_or())
        self.robots.append(robot)
        return id  # set robot.id equal to this when calling

    def reset_robots(self):
        for robot in self.robots:
            print(robot.get_start_or())
            p.resetBasePositionAndOrientation(robot.get_id(), robot.get_start_pos(), robot.get_start_or())

    # moves all robots in simulation to their starting positions

    def pass_time(self, num_steps):
        """
        args:
            None
        Output:
            This runs a simulation with no motion - hello world example
        """

        for i in range(num_steps):
            # iterate through robots, make joints go to desired target position
            for robot in self.robots:
                # robot computes its target position
                robot.compute_target_position(self.current_time)
                for x in range(0,12):
                    print(x)
                    print(robot.target_position)
                    p.setJointMotorControl2(bodyIndex=robot.id, jointIndex=x, controlMode=p.POSITION_CONTROL,
                                            targetPosition=robot.target_position[x])
            p.stepSimulation()
            time.sleep(1. / 240.)  # This is to make it more realistic - shouldn't use when training (I think)
            self.current_time += self.TIME_STEP


if __name__ == "__main__":
    print("Starting Simulation")
    my_sim = Simulator()
    current_dir = os.getcwd()
    urdf = current_dir + os.sep + os.path.join("URDF", "Ghost", "urdf", "Ghost.urdf")
    my_robot = Robot(urdf, (0, 0, 0.27))
    #my_robot_2 = Robot(urdf, (1, 0, 0.27))
    my_robot.set_id(my_sim.load_new_robot_urdf(my_robot))
    my_sim.pass_time(200) # Let robot settle in idle mode
    my_sim.reset_robots()
    #my_robot_2.set_id(my_sim.load_new_robot_urdf(my_robot_2))
    my_sim.pass_time(200) # let robot settle in idle once more
    #my_sim.reset_robots()
    #time.sleep(1)
    my_sim.robots[0].set_mode("walk")
    my_sim.pass_time(500)
    my_sim.robots[0].set_mode("idle")
    my_sim.pass_time(500)
    my_sim.reset_robots()
    my_sim.pass_time(500)
