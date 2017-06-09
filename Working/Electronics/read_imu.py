# Simple Adafruit BNO055 sensor reading example.
# Prints the orientation and calibration data to a fifo every second.

import logging
import sys
import time
import os
from Adafruit_BNO055 import BNO055
#import BNO055
print "Imported Adafruit BNO055"

# Create and configure the BNO sensor connection.
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

# what does this do?
#cproc = subprocess.Popen("./new_test", stdin=PIPE, stdout=PIPE)

# Enable verbose debug logging if -v is passed as a parameter.
#if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
#    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
print "Initializing IMU"
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
print "IMU Initialized"

# Print system status and self test result.
"""Return a tuple with status information.  Three values will be returned:
  - System status register value with the following meaning:
      0 = Idle
      1 = System Error
      2 = Initializing Peripherals
      3 = System Initialization
      4 = Executing Self-Test
      5 = Sensor fusion algorithm running
      6 = System running without fusion algorithms
  - Self test result register value with the following meaning:
      Bit value: 1 = test passed, 0 = test failed
      Bit 0 = Accelerometer self test
      Bit 1 = Magnetometer self test
      Bit 2 = Gyroscope self test
      Bit 3 = MCU self test
      Value of 0x0F = all good!
  - System error register value with the following meaning:
      0 = No error
      1 = Peripheral initialization error
      2 = System initialization error
      3 = Self test result failed
      4 = Register map value out of range
      5 = Register map address out of range
      6 = Register map write error
      7 = BNO low power mode not available for selected operation mode
      8 = Accelerometer power mode not available
      9 = Fusion algorithm configuration error
     10 = Sensor configuration error
If run_self_test is passed in as False then no self test is performed and
None will be returned for the self test result.  Note that running a
self test requires going into config mode which will stop the fusion
engine from running.
"""
status, self_test, error = bno.get_system_status()
print('IMU status: {0}'.format(status))
print('IMU Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))

# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
#print('Software version:   {0}'.format(sw))
#print('Bootloader version: {0}'.format(bl))
#print('Accelerometer ID:   0x{0:02X}'.format(accel))
#print('Magnetometer ID:    0x{0:02X}'.format(mag))
#print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
while True:
  # Read the Euler angles for heading, roll, pitch (all in degrees).
  heading, roll, pitch = bno.read_euler()

  # Gyroscope data (in degrees per second):
  p,q,r = bno.read_gyroscope()

  # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
  sys, gyro, accel, mag = bno.get_calibration_status()

  # Linear acceleration data (i.e. acceleration from movement, not gravity--
  # returned in meters per second squared):
  x_acc,y_acc,z_acc = bno.read_linear_acceleration()

  _string = "%f %f %f %f %f %f %i %i %i %i %f %f %f" %(heading, roll, pitch, p, q, r, sys, gyro, accel, mag, x_acc, y_acc, z_acc)
  # fifo = open("imu.fifo", "w")
  fifo = open("imu.fifo", "w")
  fifo.write(_string)
  fifo.close()

  # wtf does this do?
  #out, err = cproc.communicate(input)

  # Print everything out.
  # print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(heading, roll, pitch, sys, gyro, accel, mag))

  # Other values you can optionally read:
  # Orientation as a quaternion:
  #x,y,z,w = bno.read_quaterion()
  # Sensor temperature in degrees Celsius:
  #temp_c = bno.read_temp()
  # Magnetometer data (in micro-Teslas):
  #x,y,z = bno.read_magnetometer()
  # Accelerometer data (in meters per second squared):
  #x,y,z = bno.read_accelerometer()
  # Linear acceleration data (i.e. acceleration from movement, not gravity--
  # returned in meters per second squared):
  # x_acc,y_acc,z_acc = bno.read_linear_acceleration()
  # Gravity acceleration data (i.e. acceleration just from gravity--returned
  # in meters per second squared):
  #x,y,z = bno.read_gravity()

  # Sleep until the next reading.
  time.sleep(0.01)
