import time
import board
import busio
import adafruit_bno055

# Use these lines for I2C
#i2c = busio.I2C(board.SCL, board.SDA)
#sensor = adafruit_bno055.BNO055_I2C(i2c)

# User these lines for UART
#uart = busio.UART(board.TX, board.RX)
import serial
uart = serial.Serial('/dev/serial0', baudrate=9600, timeout=10)
#uart = serial.Serial('/dev/serial0')
#sensor = bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
sensor = adafruit_bno055.BNO055_UART(uart)

while True:

    print("Temperature: {} degrees C".format(sensor.temperature))
    print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print("Euler angle: {}".format(sensor.euler))
    print("Quaternion: {}".format(sensor.quaternion))
    print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    print("Gravity (m/s^2): {}".format(sensor.gravity))
    print()
    time.sleep(1)
