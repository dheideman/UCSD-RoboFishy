/******************************************************************************
 * sensor_read.cpp
 *
 * Reads pressure data from the MS5837, temperature data from the DS18B20,
 * IMU data from the BNO055, and leak sensor data from the SOS leak sensor
 *****************************************************************************/
#include "Mapper.h"

void sensor_read()
{
	// Read IMU data from the BNO055 //
	bno055 = bno055_read();

	// Read pressure data from the MS5837 //
	ms5837 = ms5837_read(pressure_calib);

	// Read temperature data from the DS18B20 //
	ds18b20 = ds18b20_read(); 	// temperature in deg C
}
