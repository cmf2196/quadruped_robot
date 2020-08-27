from lx16a import *
LX16A.initialize("/dev/ttyUSB0")
import time
from config import motor_ids

# We are only considering working  with one motor.

# Rotates servo1 to 50
#servo1.moveTimeWrite(80, 1000)
for motor in motor_ids:
	s = LX16A(motor)
	pos = s.getPhysicalPos()
	print("The servo, {} , initial physical position is {} degrees".format(motor , pos))

# Rotates servo1 to its halfway position
#servo1.moveTimeWrite(120, 1000)


#pos2 = servo1.getPhysicalPos()
#print("The servo's second physical position is {} degrees".format(pos2))

