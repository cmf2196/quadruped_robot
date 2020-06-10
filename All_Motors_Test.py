from lx16a import *
from math import sin, cos
import time

# This is the port that the controller board is connected to
# This will be different for different computers
# On Windows, try the ports COM1, COM2, COM3, etc...
# On Raspbian, try each port in /dev/
LX16A.initialize("COM3")  # /dev/ttyUSB0

print("Successfully initialized")

# There should two servos connected, with IDs 1 and 2
#servo1 = LX16A(21)
#servo2 = LX16A(22)
#servo3 = LX16A(23)

delay = 2000

servos = []
for i in range(1,5):
    for j in range(1,4):
        code = eval(str(i)+str(j))
        print(code)
        servos.append(LX16A(code))

# Turn on servos
for servo in servos:
    servo.loadOrUnloadWrite(1)

print("Turned on motors")

#for x in range(0,12):
#    for y in range(0,100):
#        print(servos[x].IDRead())

#for servo in servos: # Makes robot not work?
#    if str(servo.IDRead())[1] == "3":
#        servo.angleLimitWrite(60, 180)


stand_data = []
for servo in servos:
    id = str(servo.IDRead())
    print(id)
    if id[1] == "2":
        if int(id[0]) % 2 == 0:
            stand_data.append((85, delay))
        else:
            stand_data.append((165, delay))
    else:
        stand_data.append((120, delay))

print("generated stand data")

lay_data = []
for servo in servos:
    id = str(servo.IDRead())
    if id[1] == "2":
        if int(id[0]) % 2 == 0:
            lay_data.append((60, delay))
        else:
            lay_data.append((180, delay))
    elif id[1] == "3":
        if int(id[0]) %2 == 0:
            lay_data.append((180,delay))
        else:
            lay_data.append((60,delay))
    else:
        lay_data.append((120, delay))



for servo in servos:
    print(str(servo.IDRead())+ ": " + str(servo.getPhysicalPos()))
#down_data = []
#for servo in servos:
#    id = str(servo.IDRead())



#for servo in servos:
#    servo.moveTimeWrite(120,3000)
for x in range(0,3):
    LX16A.moveTimeWriteList(servos, stand_data)
    time.sleep(delay/1000)
    LX16A.moveTimeWriteList(servos, lay_data)
    time.sleep(delay/1000)

LX16A.moveTimeWriteList(servos, stand_data)
time.sleep(delay/1000)

#moveTimeWriteList(servos, data)


# Turn off servos
#for servo in servos:
#    servo.loadOrUnloadWrite(0)

exit()

'''
t = 0

print("Created motor objects")
# servo1.moveTimeWrite(120,3000)

delay = 3000
servo1.moveTimeWrite(120, delay)
servo2.moveTimeWrite(85, delay)
servo3.moveTimeWrite(120, delay)

time.sleep(delay / 1000)

print("Finished")

exit()
'''
'''
prev_time = time.time()

delay2 = 0
for x in range(0, 1000):

    servo3.moveTimeWrite(sin(-0.75 * t) * 40 + 40)
    # servo2.moveTimeWrite(sin(t) * 40 + 100, delay2)
    if x > 250:
        servo2.moveTimeWrite(sin(0.5 * t) * 20 + 180)
    # time.sleep(delay2/1000)

    if x > 500:
        servo1.moveTimeWrite(sin(0.25 * t) * 20 + 165, delay2)

    t += 0.2  # 0.01 is max speed\
    time.sleep(0.02 - time.time() % 0.02)  # 100 Hz
    print(time.time() - prev_time)
    prev_time = time.time()

servo1.moveTimeWrite(220, delay)
servo2.moveTimeWrite(200, delay)
servo3.moveTimeWrite(240, delay)

time.sleep(delay / 1000)'''
