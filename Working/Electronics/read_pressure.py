#!/usr/bin/python
import ms5837
import time

sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)
#sensor = ms5837.MS5837_30BA(0) # Specify I2C bus
#sensor = ms5837.MS5837_02BA()
#sensor = ms5837.MS5837_02BA(0)
#sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0) # Specify model and bus

# We must initialize the sensor before reading it
if not sensor.init():
    print "Sensor could not be initialized"
    exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print "Sensor read failed!"
    exit(1)

# print("Pressure: %.2f atm  %.2f Torr  %.2f psi") % (
# sensor.pressure(ms5837.UNITS_atm),
# sensor.pressure(ms5837.UNITS_Torr),
# sensor.pressure(ms5837.UNITS_psi))

#print("Temperature: %.2f C  %.2f F  %.2f K") % (
#sensor.temperature(ms5837.UNITS_Centigrade),
#sensor.temperature(ms5837.UNITS_Farenheit),
#sensor.temperature(ms5837.UNITS_Kelvin))

freshwaterDepth = sensor.depth() # default is freshwater
sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
saltwaterDepth = sensor.depth() # No nead to read() again
sensor.setFluidDensity(1000) # kg/m^3
#print("Depth: %.3f m (freshwater)  %.3f m (saltwater)") % (freshwaterDepth, saltwaterDepth)

#time.sleep(5)

# Spew readings
while True:
    if sensor.read():
        # write pressure info to fifo
        fifo = open("pressure.fifo", "w")

        _string = "%f %f" % (
        sensor.depth(),         # Default is m (no arguments)
        sensor.temperature())   # Default is degrees C (no arguments)

        fifo.write(_string)
        fifo.close()

#        print("Depth: {:2f} m, Temp:{:2f} C".format(sensor.depth(), sensor.temperature()))
        time.sleep(0.1)

    else:
        print "Sensor read failed!"
        # write zeros to fifo
        fifo = open("pressure.fifo", "w")
        _fail = "%f %f" % (0, 0)
        fifo.write(_fail)
        fifo.close()

        exit(1)
