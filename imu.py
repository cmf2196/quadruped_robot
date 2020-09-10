'''
Author: Connor Finn
Date: 9/ 20/ 2020


For our purposes it will only be feasible to calibrate the gyroscope
The accelerometer and magnetometer required controlled motion in order
to calibrate.

This sensor does not 'play well' with UART; however, the Pi's capabilities 
don't allow I2C connection due to something called clock stretching.To remedy
this, we will catch the RuntimeError (UART read error: 7)

The class has a method get_data()

'''



import time
import adafruit_bno055
import serial




class IMU():


  def __init__(self):
    self.uart = serial.Serial('/dev/serial0', baudrate=9600, timeout=10)
    self.sensor = adafruit_bno055.BNO055_UART(self.uart)



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
      data += [self.linear_acceleration()]
    
    except RuntimeError:
      return 0

  def get_euler_angles(self):
    return self.sensor.euler

  def get_temperature(self):
    return self.temperature 

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
   

  for i in range(runs):
    new_data = imu.get_data()
    if new_data != 0:            # if we get a reading
      data = new_data

    else:
      missed += 1
    print(data)

    time.sleep(1)

  print('percent readings missed: ' , missed / runs )