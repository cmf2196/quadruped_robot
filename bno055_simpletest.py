'''
Author: Connor Finn
Date: 9/ 20/ 2020


For our purposes it will only be feasible to calibrate the gyroscope
The accelerometer and magnetometer required controlled motion in order
to calibrate.

This sensor does not 'play well' with UART; however, the Pi's capabilities 
don't allow I2C connection due to something called clock stretching.To remedy
this, we will catch the RuntimeError (UART read error: 7)


'''







import time
import adafruit_bno055
import serial





uart = serial.Serial('/dev/serial0', baudrate=9600, timeout=10)
sensor = adafruit_bno055.BNO055_UART(uart)



if __name__ == "__main__":

   while True:

       try:
          print("Temperature: {} degrees C".format(sensor.temperature))
          print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
          print("Magnetometer (microteslas): {}".format(sensor.magnetic))
          print("Gyroscope (rad/sec): {}".format(sensor.gyro))
          print("Euler angle: {}".format(sensor.euler))
          print("Quaternion: {}".format(sensor.quaternion))
          print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
          print("Gravity (m/s^2): {}".format(sensor.gravity))
          print("calibration " , sensor.calibration_status)
          print()
    
       except RuntimeError:
          print('missed one')
          pass

       time.sleep(1)
