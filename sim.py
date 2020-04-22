import os  # we could use this to get the path to the urdf files if you want to put them elsewhere
import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import statistics

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
        #print(cube_pos, cube_orn)

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

    ''' This is a primitive function for computing a target position for a joint based on the parameters for a sine
    wave. Input is A,B,C,W,t, and output is a floating point number.
    '''

    def compute_target(self, A, B, C, W, t):
        return A * np.sin(t * W + B) + C

    def walk(self):
        """
        Args:
            None
        Output:
            This runs a simulation of a sine wave
        """

        # Array to record walking joint information
        walk_data = []

        # array of sinusoid constants in form A*sin(wt+B)+C. Array is [[A1, B1, C1], [A2, B2, C2], ... ,[An, Bn, Cn]]
        parameters = []

        # Parameters for now are filled in by hand. Begin by creating array of 0's
        for x in range(0, 12):
            parameters.append([0, 0, 0])  # [A,B,C]

        # Next I will adjust A for each group. I assume that all motors of same type have same amplitude. Therefore,
        # there are three amplitudes:

        A_1 = 0
        A_2 = np.pi / 4.0  # just a guess, 90 deg is not a bad place to start
        A_3 = 0  # np.pi / 4.0

        # Next we need B values. I assume half will be 0 and half will be ~pi.

        # Finally we need C values. For now I will assume these are all 0.

        # So now let's modify the paramater list:

        # Amplitudes:
        for x in range(0, 12, 3):  # modifying every third motor of type 1
            parameters[x][0] = A_1
        for x in range(1, 12, 3):  # modifying every third motor of type 2
            parameters[x][0] = A_2
            # if x == 1 or x == 4:
            #    parameters[x][1] = np.pi
        for x in range(2, 12, 3):  # modifying every third motor of type 3
            parameters[x][0] = A_3

        # showing parameters for debugging purposes
        #print(parameters)

        # initialize variables: A * sin(omega * t + phi)

        A = np.pi / 4.0  # amplitude
        omega = 0.03  # .0628  # Angular Frequency: Consistent for all limbs ?
        phi = 0  # Phase: This is going to need to be adjusted for each motor
        phi1 = 3 * np.pi / 4.0

        # record velocity to find average velocity
        base_velocity = []

        for t in range(1000):

            for x in range(1, 12, 3):
                p.setJointMotorControl2(bodyIndex=self.r_id, jointIndex=x, controlMode=p.POSITION_CONTROL,
                                        targetPosition=self.compute_target(parameters[x][0], parameters[x][1],
                                                                           parameters[x][2], omega, t))

            '''target = A * np.sin(t * omega + phi)
            target2 = -A * np.sin(t * omega + phi)

            target3 = A * np.sin(t * omega + phi1)
            target4 = -A * np.sin(t * omega + phi1)

            p.setJointMotorControl2(bodyIndex=self.r_id, jointIndex=4, controlMode=p.POSITION_CONTROL,
                                    targetPosition=target)

            p.setJointMotorControl2(bodyIndex=self.r_id, jointIndex=1, controlMode=p.POSITION_CONTROL,
                                    targetPosition=target)

            p.setJointMotorControl2(bodyIndex=self.r_id, jointIndex=7, controlMode=p.POSITION_CONTROL,
                                    targetPosition=target2)

            p.setJointMotorControl2(bodyIndex=self.r_id, jointIndex=10, controlMode=p.POSITION_CONTROL,
                                    targetPosition=target2)'''

            # record joint information
            walk_data.append(p.getJointStates(self.r_id, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]))
            base_velocity.append(p.getBaseVelocity(self.r_id)[0][1])


            p.stepSimulation()
            time.sleep(1. / 240.)  # This is to make it more realistic - shouldn't use when training (I think)

        print("The average linear velocity was: " + str(statistics.mean(base_velocity))+ " m/s")
        return walk_data

    def plot_motor_data(self, motor_id, data):
        position = []
        velocity = []
        torque = []
        t = []

        for i in range(0, len(data)):
            time_step_data = data[i][motor_id]
            position.append(time_step_data[0])
            velocity.append(time_step_data[1])
            torque.append(time_step_data[3])
            t.append(i)

        plt.plot(t, position)
        plt.xlabel("Time (ms)")
        plt.ylabel("Position (rad)")
        plt.title("Position for Motor " + str(motor_id)+ " vs. Time")
        plt.show()
        plt.plot(t, velocity)
        plt.xlabel("Time (ms)")
        plt.ylabel("Velocity (rad/s)")
        plt.title("Velocity for Motor " + str(motor_id)+ " vs. Time")
        plt.show()
        plt.plot(t, torque)
        plt.xlabel("Time (ms)")
        plt.ylabel("Torque (N*m)")
        plt.title("Torque for Motor " + str(motor_id)+ " vs. Time")
        plt.show()


if __name__ == '__main__':
    start_pos = [0, 0, .27]  # indicate starting position and orientation
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # " "
    current_dir = os.getcwd()
    urdf = current_dir + os.sep + os.path.join("URDF","Ghost","urdf","Ghost.urdf")#"\\URDF\\Ghost\\urdf\\Ghost.urdf"
    path = "Ghost/urdf/Ghost.urdf"  # indicate path to robot urdf file
    s = Simulator(urdf, start_pos, start_orientation)  # initialize Simulator object
    s.go_to_start()
    s.pass_time(300)
    # let the robot settle
    #s.jump()
    # s.pass_time(200)
    s.pass_time(200)
    walk_data = s.walk()
    s.plot_motor_data(0,walk_data)
    #print("The joint information is:")
    #print(p.getJointStates(s.r_id, [1, 2, 3]))

    # Run a simulation
    p.disconnect
