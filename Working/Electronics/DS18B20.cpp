/******************************************************************************
*DS18B20.cpp
*
*File to run Initialization and reading files on the BNO055 IMU
******************************************************************************/
#include "Mapper.h"






///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// DS18B20 Functions ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/***************************************************************************
 * void start_Py_ds18b20
 *
 * Writes temperature values from the DS18B20 temperature sensor into
 * ds18b20_fifo.fifo
***************************************************************************/

void start_Py_ds18b20(void)		//	start temperature_sensor_code.py code
{
    std::FILE* fd = fopen("temperature_sensor_code.py", "r");
	PyRun_SimpleFile(fd,"temperature_sensor_code.py");
	return;
}

/***************************************************************************
 * ds18b20_t ds18b20_read(void)
 *
 * Reads temperature values from ds18b20_fifo.fifo
***************************************************************************/

ds18b20_t ds18b20_read(void)	// read values from ds18b20 temperature sensor
{
	ds18b20_t ds18b20;
	char buf[1000];
    std::FILE *fd = fopen( "ds18b20_fifo.fifo", "r");
	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f",&ds18b20.temperature);
	return ds18b20;
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////Setup and Shutdown//////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
