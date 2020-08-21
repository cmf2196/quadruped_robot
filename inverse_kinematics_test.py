"""
This is the example code provided by Dr. Lipson in class
"""
import os  # we could use this to get the path to the urdf files if you want to put them elsewhere
import pybullet as p
import time
import pybullet_data
import math

current_dir = os.getcwd()
# urdf = current_dir + "\\URDF\\Ghost\\urdf\\Ghost.urdf"
urdf = current_dir + "\\Phantom\\urdf\\Phantom_connor_edits.urdf"
print(urdf)

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeID = p.loadURDF("plane.urdf")  # Floor
# Starting position and orientation
cube_start_pos = [0, 0, 1]
cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
# import our urdf file
Phantom_IK = p.loadURDF(urdf, cube_start_pos, cube_start_orientation)
Phantom = p.loadURDF(urdf, [1, 0, 1], cube_start_orientation)

# get end effector link index

# Compute inverse kinematics for one leg

# p.calculateInverseKinematics(box_ID, )
print("Diagnostic Info About URDF:")
num_joints = p.getNumJoints((Phantom_IK))
print(num_joints)

joint_info = []
for i in range(0, num_joints):
    joint_info.append(p.getJointInfo(Phantom_IK, i))
    print(joint_info[i])

link_states = p.getLinkStates(Phantom_IK, [3, 7, 11, 15])
print("Link States:")
for entry in link_states:
    print(entry[0])
command = p.calculateInverseKinematics(Phantom_IK, 3, [0.13495, -0.30683, (0.5)])
print("ik result:")
print(command)

command2 = p.calculateInverseKinematics2(Phantom_IK, [3, 7, 11, 15], [[0.13495, -0.306835, 1],
                                                                      [-0.13495, -0.306835, 1],
                                                                      [0.13495, -0.0156337, 1],
                                                                      [-0.13495, -0.0156337, 1]])
print(command2)

print("")
moving_joints = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]

# forward kinematics test


# begin the simulation
for i in range(1000000):
    command3 = p.calculateInverseKinematics2(Phantom_IK, [3, 7, 11, 15], [
        [0.05 * math.sin(0.01 * i+math.pi/2) + 0.13495, 0.05 * math.sin(0.01 * i) - 0.206835+0.1, 0.80],
        [0.05 * math.sin(0.01 * i+math.pi/2) -0.13495, 0.05 * math.sin(0.01 * i)-0.306835+0.1, 0.80],
        [0.05 * math.sin(0.01 * i+math.pi/2) +0.13495, 0.05 * math.sin(0.01 * i)-0.0156337+0.1, 0.80],
        [0.05 * math.sin(0.01 * i+math.pi/2) -0.13495, 0.05 * math.sin(0.01 * i)-0.0156337+0.1, 0.80]], maxNumIterations=50)
    p.resetBasePositionAndOrientation(Phantom_IK, cube_start_pos, cube_start_orientation)

    for i in range(0, len(moving_joints)):
        p.setJointMotorControl2(bodyIndex=Phantom, jointIndex=moving_joints[i], controlMode=p.POSITION_CONTROL,
                                targetPosition=command3[i])

    # p.setJointMotorControl2(bodyIndex=box_ID, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=command[0])
    # p.setJointMotorControl2(bodyIndex=box_ID, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=command[1])
    # p.setJointMotorControl2(bodyIndex=box_ID, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=command[2])

    p.stepSimulation()
    time.sleep(1. / 240.)  # This is to make it more realistic - shouldn't use when training (I think)
cube_pos, cube_orn = p.getBasePositionAndOrientation(Phantom_IK)
print(cube_pos, cube_orn)
p.disconnect()  # good practice
