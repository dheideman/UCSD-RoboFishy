# Simple Adafruit BNO055 sensor reading example.  Will print the orientation
# and calibration data every second.
#
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import logging
import sys
import time
import os
#from Adafruit_BNO055 import BNO055
import BNO055
print "Imported everything"

# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
#fifo_path = "/tmp/bno055_fifo2.fifo"
#fifo_path = "/home/pi/Documents/156b/feedback2/bno055_fifo.fifo"
#os.mkfifo(fifo_path)

#cproc = subprocess.Popen("./new_test", stdin=PIPE, stdout=PIPE)
# BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
# and RST connected to pin P9_12:
#bno = BNO055.BNO055(rst='P9_12')


# Enable verbose debug logging if -v is passed as a parameter.
#if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
#    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
#if not bno.begin():
imu_initialized = False
print "Initializing IMU..."
counter = 0;
start_time = time.time()
counter = 0
while not imu_initialized:
    counter += 1
    print "Counter: ", counter
    imu_initialized = bno.begin()
    print(imu_initialized)
    counter += 1
    #print "Counter: ", counter
    if (time.time() - start_time) > 10:
        # print failure statement if IMU not intiialized after 10 seconds
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
        break

#bno.begin()
#time.sleep(0.002)
#bno.begin()

#if not bno.begin():
#    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')


# Print system status and self test result.
status, self_test, error = bno.get_system_status()
#print('System status: {0}'.format(status))
#print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
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

#print('Reading BNO055 data, press Ctrl-C to quit...')
while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
	# Gyroscope data (in degrees per second):
    p,q,r = bno.read_gyroscope()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    _string = "%f %f %f %f %f %f %i %i %i %i" %(heading, roll, pitch, p, q, r, sys, gyro, accel, mag)
    fifo = open("bno055_fifo.txt", "w")
    #_string = "%f %f %f %f %f %f %i %i %i %i" %(heading, roll, pitch, p, q, r, sys, gyro, accel, mag)
    fifo.write(_string)
    fifo.close()
#    print "%f %f %f %f %f %f %i %i %i %i\n" %(heading, roll, pitch, p, q, r, sys, gyro, accel, mag)

    #out, err = cproc.communicate(input)
    # Print everything out.
    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
    #      heading, roll, pitch, sys, gyro, accel, mag))
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
    #x,y,z = bno.read_linear_acceleration()
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()
    # Sleep for a second until the next reading.
    #time.sleep(0.001)

