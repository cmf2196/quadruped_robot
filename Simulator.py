"""
This module contains a class for simulating robot urdfs

Joshua Katz
8/28/20
"""

import os
import time
import pybullet_utils.bullet_client as bc
import pybullet
import pybullet_data


class Simulator:

    def __init__(self, gui=False):

        # connect to pybullet with two seperate clients
        if gui is True:
            self.gui_sim = bc.BulletClient(connection_mode=pybullet.GUI)
            self.gui_sim.setAdditionalSearchPath(pybullet_data.getDataPath())
            self.gui_plane = self.gui_sim.loadURDF("plane.urdf")
            self.gui_sim.setGravity(0, 0, -9.8)
            self.gui_sim.setTimeStep(1 / 100)  # change to variable
        else:
            self.gui_sim = None
            self.gui_plane = None

        self.kinematics_sim = bc.BulletClient(connection_mode=pybullet.DIRECT)
        self.kinematics_sim.setAdditionalSearchPath(pybullet_data.getDataPath())
        # self.kinematics_sim.setGravity(0, 0, -9.8)

        self.kinematics_robot = None
        self.gui_robot = None

        self.gravity = (0, 0, -9.8)  # default gravity value
        self.current_time = 0
        self.TIME_STEP = 1
        self.gui = gui

    def load_kinematics_urdf(self, urdf):
        if self.kinematics_robot is not None:
            print("Delete robot! (NEED TO IMPLEMENT THIS)")
        start_orientation = self.gui_sim.getQuaternionFromEuler([0, 0, 3.14159])
        self.kinematics_robot = self.kinematics_sim.loadURDF(urdf, (0, 0, 0),
                                                             pybullet.getQuaternionFromEuler(
                                                                 (0, 0,
                                                                  3.14159)))

        self.kinematics_sim.resetBasePositionAndOrientation(
            self.kinematics_robot,
            (0, 0, 0),
            start_orientation)
        return

    def load_gui_urdf(self, urdf):
        if not self.gui:
            print("GUI not active")
            return

        if self.gui_robot is not None:
            print("Delete robot! (NEED TO IMPLEMENT THIS)")

        self.gui_robot = self.gui_sim.loadURDF(urdf, (0, 0, 0.4),
                                               pybullet.getQuaternionFromEuler(
                                                   (0, 0, 3.14159)))
        return

    def compute_ik(self, leg_id, coord):
        ik = self.kinematics_sim.calculateInverseKinematics(
            self.kinematics_robot,
            leg_id, coord)
        return ik

    def compute_multi_ik(self, leg_ids, coords):
        # self.kinematics_sim.resetBasePositionAndVelocity(self.kinematics_robot, (0,0.1,0), pybullet.getQuaternionFromEuler(
        #                                           (0, 0, 3.14159)))
        # self.kinematics_sim.stepSimulation()
        ik = self.kinematics_sim.calculateInverseKinematics2(
            self.kinematics_robot,
            leg_ids, coords, maxNumIterations=50)

        # new addition: move ik robot to position to make future ik computations
        # potentially faster
        moving_joints = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
        for i in range(0, len(moving_joints)):
            self.kinematics_sim.resetJointState(self.kinematics_robot,
                                                moving_joints[i],
                                                ik[i], 0)

        # it = iter(ik)
        # ik = list(zip(it, it, it))
        return ik

    def set_robot_pos(self, joints, command):
        if not self.gui:
            print("GUI not active")
            return
        self.gui_sim.setJointMotorControlArray(bodyIndex=self.gui_robot,
                                               jointIndices=joints,
                                               controlMode=pybullet.POSITION_CONTROL,
                                               targetPositions=command)
        return

    def step_gui_sim(self):
        if not self.gui:
            print("GUI not active")
            return
        self.gui_sim.stepSimulation()
        return

    def reset_gui_robot(self):
        self.gui_sim.resetBasePositionAndOrientation(self.gui_robot,
                                                     (0, 0, 0),
                                                     pybullet.getQuaternionFromEuler(
                                                         (0, 0, 3.14159)))
        return


if __name__ == "__main__":
    current_dir = os.getcwd()
    my_urdf = current_dir + "\\Phantom\\urdf\\Phantom_connor_edits.urdf"

    sim = Simulator(True)
    sim.load_gui_urdf(my_urdf)
    sim.load_kinematics_urdf(my_urdf)

    for x in range(0, 1000000):
        sim.step_gui_sim()
        start_time = time.time()
        command = sim.compute_multi_ik([3, 7, 11, 15], (
            (-0.15, 0.135, -0.2), (0.15, 0.135, -0.2), (-0.15, -0.135, -0.2),
            (0.15, -0.135, -0.2)))

        sim.set_robot_pos([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14], command)
        print(time.time() - start_time)
        time.sleep(1 / 240)
