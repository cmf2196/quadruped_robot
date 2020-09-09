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

    def __init__(self, motor_ids, frequency):
        self.initialize_connection()

        # create array of servo objects
        self.motor_ids = motor_ids
        self.servos = [LX16A(motor) for motor in motor_ids]
        self.frequency = frequency
        self.period = 1/frequency

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
        data = [(angle, t) for angle in angles]
        self.servos[0].moveTimeWriteList(self.servos, data)

    # this converts the radian positions of the robot ik output to degrees in
    # the motor's reference. Center of input is 0, center of motor is 120
    def radians_to_degrees(self, rads):
        degs = [rad/(2*math.pi)*360 + 120 for rad in rads]
        return degs

if __name__ == "__main__":
    ids = []
    for i in range(1, 5):
        for j in range(1, 4):
            code = eval(str(i) + str(j))
            print(code)
            ids.append(code)

    motor_controller = MotorController(ids, 100)
    motor_controller.turn_all_motors_on()

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

