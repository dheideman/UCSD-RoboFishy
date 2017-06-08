###############################################################################
# Temperature_Sensor_Functions.cpp
#
# Reads temperature data from the DS18B20 temperature sensor and writes it to
# temp.fifo
# https://learn.adafruit.com/adafruits-raspberry-pi-lesson-11-ds18b20
# -temperature-sensing/software
###############################################################################

import os
import glob
import time
# uncomment if using alternative read_temp_raw()
#import subprocess

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

"""
If you're having problems I feel bad for you son...whoops.
If you're having problems reading temp, try replacing "read_temp_raw()" with
def read_temp_raw():
	catdata = subprocess.Popen(['cat',device_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	out,err = catdata.communicate()
	out_decode = out.decode('utf-8')
	lines = out_decode.split('\n')
	return lines
"""

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        temp_f = temp_c * 9.0 / 5.0 + 32.0
        #return temp_c, temp_f
        #return temp_c


# write temp info to fifo
while True:
    temp_c = read_temp()
    fifo = open("temp.fifo", "w")
    _string = "%f" %(temp_c)
    fifo.write(_string)
    fifo.close()
    print "%f\n" %(temp_c)
    time.sleep(1)
