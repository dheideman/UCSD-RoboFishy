/******************************************************************************
 * SensorRead .cpp
 *
 * Reads pressure data from the MS5837, temperature data from the DS18B20, 
 * IMU data from the BNO055, and leak sensor data from the SOS leak sensor
 *****************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * pressure_calib_t init_ms5837
 *
 * initializes pressure sensor
 * Returns structure of 6 coefficients
***************************************************************************/

float *sensorread_thread(void* arg)
{
	// Read IMU data from the BNO055 //
	bno055 = bno055_read();

	// Read pressure data from the MS5837 //


	// Read temperature data from the DS18B20 //
	ds18b20 = ds18b20_read(); 	// temperature in deg C

	// Read leak sensor data from the SOS leak sensor //
	

}