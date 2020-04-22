import os  # we could use this to get the path to the urdf files if you want to put them elsewhere
import pybullet as p
import time
import pybullet_data
import numpy as np

# _________________ Init environment

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
planeID = p.loadURDF("plane.urdf")  # Floor


# ________________ Robot Starting Location and Orientation


class Simulator():

    def __init__(self, path, pos, orientation):
        """
        args:
            path: String - This is the path to the robot URDF on your local computer
            pos: List - This is a list of three float values, [x , y , z]  This gives the starting position
            orientation: quaternion - this gives the robots starting orientation
                I reccomend using the getQuaternionFromEuler Pybullet function
        """
        self.start_pos = pos
        self.cube_start_orientation = orientation
        self.r_id = p.loadURDF(path, pos, orientation)
        self.num_joints = p.getNumJoints(self.r_id)
        self.joint_name_id = {}
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.r_id, i)
            self.joint_name_id[joint_info[1].decode('UTF-8')] = joint_info[0]

    def move_arm(self):
        """
        This is obsolete, I left it in here so that we had the work flow for inverse kinematics.
        DO NOT USE AS IS

        """

        target = p.calculateInverseKinematics(self.r_id, 1, [0, 2.05, 0])

        p.setJointMotorControl2(bodyIndex=self.r_id,
                                jointIndex=0,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target[0])

    def pass_time(self, num_steps):
        """
        args:
            None
        Output:
            This runs a simulation with no motion - hello world example
        """

        for i in range(num_steps):
            p.stepSimulation()
            time.sleep(1. / 240.)  # This is to make it more realistic - shouldn't use when training (I think)
        cube_pos, cube_orn = p.getBasePositionAndOrientation(self.r_id)
        print(cube_pos, cube_orn)

    def move_joints(self, joint_locs, num_seconds):
        """
        Description:
            This function takes in the desired joint locations and a number of simulation steps.
            The robot then moves to those joint positions in that period of time

        Args:
            joint_locs: numpy array - an array of joint locations
            num_seconds: int - the desired number of simulation steps


        Output:
            This function does not return anything, but it moves the robot to the indicated joint positions
            in the indicated amount of time

        """

        # Get the current joint positions

        curr_pos = np.zeros(self.num_joints)  # init an array
        joints = list(range(self.num_joints))  # Get the joint indices

        vals = p.getJointStates(self.r_id, joints)  # get the joint state information
        for i in range(self.num_joints):  # Fill the curr_pos array with current joint positions
            curr_pos[i] = vals[i][0]  # "  "

        # Get the stepping distance
        target = joint_locs - curr_pos  # find the change in joint states
        step = np.true_divide(target, num_seconds)  # divide this by the desired number of steps

        # move the robot
        for j in range(num_seconds):
            next_loc = np.add(curr_pos, np.multiply(step, j + 1))  # get the next position
            p.setJointMotorControlArray(self.r_id, joints, p.POSITION_CONTROL, next_loc)  # move the joints

            p.stepSimulation()  # Execute the action
            time.sleep(1. / 240.)  # pause so that the simulation looks realistic

    def go_to_start(self):
        """
        Description:
            This function is just an application of the move_joints function. I separate it so that we can
            set the gravity after the robot has moved to it's starting position
            It should only be called one time and at the very begining of the program
        Args:
            None
        Output:
            None, it simply sets the robot to the starting position and FAST
        """

        # args for self.move_joints()
        num_seconds = 100
        goal = np.array((0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0),
                         0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0)))

        self.move_joints(goal, num_seconds)
        p.setGravity(0, 0, -9.8)  # set gravity

    def jump(self):
        """
        To do this, the idea is to slowly lower down, then explode upward.

        """

        # Crouch Down
        goal = np.multiply(np.array((0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0),
                                     0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0))), 1.3)

        # These are the current parameters for the backflip
        adjust = 1.32
        back_flip = np.array((0, -1 * adjust * np.pi / (4.0), -1 * adjust * np.pi / (2.0), 0, adjust * np.pi / (4.0),
                              adjust * np.pi / (2.0),
                              0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0)))

        self.move_joints(back_flip, 100)

        # explode up
        self.move_joints(np.zeros(self.num_joints), 3)

        # coast
        self.pass_time(50)

        # return to start

        start = np.array((0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0),
                          0, -1 * np.pi / (4.0), -1 * np.pi / (2.0), 0, np.pi / (4.0), np.pi / (2.0)))

        self.move_joints(start, 100)
        # coast
        self.pass_time(500)

    def walk(self):
        """
        Args:
            None
        Output:
            This runs a simulation of a sine wave
        """

        # initialize variables: A * sin(omega * t + phi)

        A = np.pi / 4.0  # amplitude
        omega = .0628  # Angular Frequency: Consistent for all limbs ?
        phi = 0  # Phase: This is going to need to be adjusted for each motor
        phi1 = 3 * np.pi / 4.0

        for t in range(500):
            target = A * np.sin(t * omega + phi)
            target2 = -A * np.sin(t * omega + phi)

            target3 = A * np.sin(t * omega + phi1)
            target4 = -A * np.sin(t * omega + phi1)

            p.setJointMotorControl2(bodyIndex=self.r_id,
                                    jointIndex=4,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target)

            # p.setJointMotorControl2(bodyIndex=self.r_id,
            #                          jointIndex=5,
            #                          controlMode=p.POSITION_CONTROL,
            #                          targetPosition=target4 )

            p.setJointMotorControl2(bodyIndex=self.r_id,
                                    jointIndex=1,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target)

            p.setJointMotorControl2(bodyIndex=self.r_id,
                                    jointIndex=7,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target2)

            p.setJointMotorControl2(bodyIndex=self.r_id,
                                    jointIndex=10,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target2)

            p.stepSimulation()
            time.sleep(1. / 240.)  # This is to make it more realistic - shouldn't use when training (I think)


if __name__ == '__main__':
    start_pos = [0, 0, .27]  # indicate starting position and orientation
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # " "
    current_dir = os.getcwd()
    urdf = current_dir + "\\URDF\\Ghost\\urdf\\Ghost.urdf"
    path = "Ghost/urdf/Ghost.urdf"  # indicate path to robot urdf file
    s = Simulator(urdf, start_pos, start_orientation)  # initialize Simulator object
    s.go_to_start()
    s.pass_time(1000)
    # let the robot settle
    s.jump()

    # Run a simulation
    p.disconnect
