'''
Author: Connor Finn
Date: 9/ 20/ 2020


For our purposes it will only be feasible to calibrate the gyroscope
The accelerometer and magnetometer required controlled motion in order
to calibrate.

This sensor does not 'play well' with UART; however, the Pi's capabilities 
don't allow I2C connection due to something called clock stretching.To remedy
this, we will catch the RuntimeError (UART read error: 7)



after some movement / time, the calibration overall improves and good readings
can be had for euler

I havee converted the euler readings to radians. 

Currently we miss 20% of readings due to RuntimeErrors . This should be ok as 
long as we frequently call the sensor for readings.

We can likely create an automated motion for the robot in order to calibrate
the gyro. calibrating the accelerometer is not going to feasible.


'''



import time
import adafruit_bno055
import serial
from math import radians



class IMU():


  def __init__(self):
    # initialize the uart connection to the imu 
    self.uart = serial.Serial('/dev/serial0', baudrate=9600, timeout=10)
    # initialize the sensor 
    self.sensor = adafruit_bno055.BNO055_UART(self.uart)
    '''
    we will use the following state to determine whether the robot is fallen or not
        + ['Stable' , 'Stable']     ~ The robot has not fallen over: Resume normal Activity
        + ['Left' ,  ~ ]      ~ The robot fell to the left
        + ['Right' , ~ ]      ~ The robot fell to the right
        + [ ~ , 'Front']      ~ The robot fell forward
        + [ ~ ,  'Back']       ~ The robot fell backward
        
        + the robot can have fallen front and to the side as well
    '''
    self.state = ['Stable' , 'Stable']     


  def update_state(self):

    # This function checks to see if the robot has tipped over in any direction
    # it updates the imu state
    
    # Get the data in radians
    try:
      euler = self.get_euler_angles()
      # Check rotation about x axis
      if euler[1] > 1:
        self.state[0] = 'Back'
      elif euler[1] < -1:
        self.state[0] = 'Front'
      else:
        self.state[0] = 'Stable'

      # Check rotation about y axis
      if euler[2] > 1:
        self.state[1] = 'Left'
      elif euler[2] < -1:
        self.state[1] = 'Right'
      else:
        self.state[1] = 'Stable'

    except RuntimeError:
      # if there is a RuntimeError, simply return the previous state
      pass

  def get_state(self):
    self.update_state()
    return self.state

  def get_calibration_status(self):
    # get the calibration status
    # it returns system , gyro , accellerometer , magnetometer


    try:
      return self.sensor.calibration_status
    
    except RuntimeError:
      return 0


  def get_data(self):
    # This class will be used to actually call the data we want. it catches the RunTimeError
    data = []
    try:
      data += [self.get_euler_angles()]
      data += [self.get_angular_velocity()]
      data += [self.get_linear_acceleration()]
      return data
    except RuntimeError:
      return 0

  def get_euler_angles(self):
    # DEGREES 
    #(about z , about x , about y)
    return [radians(x) for x in self.sensor.euler]
    #return self.sensor.euler
  def get_temperature(self):
    return self.sensor.temperature 

  def get_angular_velocity(self):
    return self.sensor.gyro

  def get_quaternion(self):
    return self.sensor.quaternion

  def get_linear_acceleration(self):
    # This is acceleration - gravity. ( we can call acceleration and gravity separately)
    return self.sensor.linear_acceleration

  def get_magnetic(self):
    return self.sensor.magnetic



if __name__ == "__main__":


   imu = IMU()
   data = []
   runs = 50
   missed = 0
   for j in range(30):
      print('calibration: ' , imu.get_calibration_status())
      time.sleep(1)   

   for i in range(runs):
     new_data = imu.get_data()
     if new_data != 0:            # if we get a reading
       data = new_data

     else:
       missed += 1
    
   
     print('data = ' ,data)
     print('calibration = ' , imu.get_calibration_status())

     time.sleep(.5)

   print('percent readings missed: ' , missed / runs )
