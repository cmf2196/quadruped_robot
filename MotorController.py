"""
Joshua Katz
9/9/20
"""
import math
import time
import platform
from lx16a import *
import lx16a


class MotorController:

    def __init__(self, motor_ids, frequency , offsets = [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,0 ]):
        self.initialize_connection()

        # create array of servo objects
        self.motor_ids = motor_ids
        self.servos = [LX16A(motor) for motor in motor_ids]
        self.frequency = frequency
        self.period = 1 / frequency
        self.offsets = offsets 
    @staticmethod
    def initialize_connection():
        # initiate connection to motors according to os
        # numbers may have to change depending on selected port

        plat = platform.system()
        if plat == "Linux":
            LX16A.initialize("/dev/ttyUSB0")
        elif plat == "Windows":
            LX16A.initialize("COM3")
        else:
            print("Motors not functional on iOS, get a respectable computer!")
            exit()

    def turn_all_motors_on(self):
        for servo in self.servos:
            servo.loadOrUnloadWrite(1)

    def turn_all_motors_off(self):
        for servo in self.servos:
            servo.loadOrUnloadWrite(0)

    # angles in degrees!
    def move_all_motors(self, angles, t):
        adjusted_angles = [a + b for a, b in zip(angles, self.offsets)]
        data = [(angle, t) for angle in adjusted_angles]
        self.servos[0].moveTimeWriteList(self.servos, data)

    # this converts the radian positions of the robot ik output to degrees in
    # the motor's reference. Center of input is 0, center of motor is 120
    @staticmethod
    def radians_to_degrees(rads):
        degs = [rad / (2 * math.pi) * 360 + 120 for rad in rads]
        return degs

    def recover_from_fall(self):

        delay = 1000

        # first, bring all x2 and x3 motors in
        lay_data = []
        for servo in self.servos:
            id = str(servo.IDRead())
            if id[1] == "2":
                if int(id[0]) % 2 == 0:
                    lay_data.append(60)
                else:
                    lay_data.append(180)
            elif id[1] == "3":
                if int(id[0]) % 2 == 0:
                    lay_data.append(180)
                else:
                    lay_data.append(60)
            else:
                # don't move type 1 motors yet
                lay_data.append(servo.getVirtualPos())

        self.move_all_motors(lay_data, delay)
        time.sleep(delay / 1000)

        # now fold x1 motors in (might only do left or right based on fall)

        lay_data = []
        for servo in self.servos:
            id = str(servo.IDRead())
            if id[1] == "1":
                if int(id[0]) % 2 == 0:
                    lay_data.append(60)
                else:
                    lay_data.append(180)
            else:
                # don't move type 2 or 3 here
                lay_data.append(servo.getVirtualPos())

        self.move_all_motors(lay_data, delay)
        time.sleep(delay / 1000)

        # Robot should now be in a reasonable position to just transition to
        # laying and go to lay state

        # This is the quick and dirty way, could be optimized later

        lay_data = []
        for servo in self.servos:
            id = str(servo.IDRead())
            if id[1] == "2":
                if int(id[0]) % 2 == 0:
                    lay_data.append(60)
                else:
                    lay_data.append(180)
            elif id[1] == "3":
                if int(id[0]) % 2 == 0:
                    lay_data.append(180)
                else:
                    lay_data.append(60)
            else:
                lay_data.append(120)

        self.move_all_motors(lay_data, delay)
        time.sleep(delay / 1000)

        # robot should be laying
        return


if __name__ == "__main__":

    ids = []
    for i in range(1, 5):
        for j in range(1, 4):
            code = eval(str(i) + str(j))
            print(code)
            ids.append(code)

    motor_controller = MotorController(ids, 100)
    motor_controller.turn_all_motors_on()

    # new addition to test fall recovery
    motor_controller.recover_from_fall()

    delay = 2000
    stand_data = []
    for servo in motor_controller.servos:
        id = str(servo.IDRead())
        print(id)
        if id[1] == "2":
            if int(id[0]) % 2 == 0:
                stand_data.append(85)
            else:
                stand_data.append(165)
        else:
            stand_data.append(120)

    print("generated stand data")

    lay_data = []
    for servo in motor_controller.servos:
        id = str(servo.IDRead())
        if id[1] == "2":
            if int(id[0]) % 2 == 0:
                lay_data.append(60)
            else:
                lay_data.append(180)
        elif id[1] == "3":
            if int(id[0]) % 2 == 0:
                lay_data.append(180)
            else:
                lay_data.append(60)
        else:
            lay_data.append(120)

    print("generated lay data")

    for x in range(0, 3):
        motor_controller.move_all_motors(stand_data, delay)
        time.sleep(delay / 1000)

        motor_controller.move_all_motors(lay_data, delay)
        time.sleep(delay / 1000)

    motor_controller.move_all_motors(stand_data, delay)
    time.sleep(delay / 1000)

