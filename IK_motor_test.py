"""
Joshua Katz
9/10/2020
"""

import os
from Simulator import Simulator
from MotorController import MotorController

if __name__ == "__main__":

    ids = []
    for i in range(1, 5):
        for j in range(1, 4):
            code = eval(str(i) + str(j))
            print(code)
            ids.append(code)

    frequency = 100

    current_dir = os.getcwd()
    sep = os.path.sep
    my_urdf = current_dir + sep + "Phantom" + sep + "urdf" + sep + "Phantom_connor_edits.urdf"
    sim = Simulator(True)
    sim.load_kinematics_urdf(my_urdf)
    sim.load_gui_urdf(my_urdf)


    # Command robot to go to stand position using ik
    ik = sim.compute_multi_ik([3, 7, 11, 15], [(-0.135, 0.15, -0.2), (0.135, 0.15, -0.2),
                       (-0.135, -0.15, -0.2), (0.135, -0.15, -0.2)])

    print(ik)
    degs = MotorController.radians_to_degrees(ik)

    for x in range(0,len(ids)):
        print(str(ids[x]) + ": " + str(degs[x]))

    # command motors
    motor_controller = MotorController(ids, 100)
    motor_controller.move_all_motors(degs, 3000)


