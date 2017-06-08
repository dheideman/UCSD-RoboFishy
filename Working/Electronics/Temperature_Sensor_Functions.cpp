/******************************************************************************
* Temperature_Sensor_Functions.cpp
*
* Runs initialization and reads files on the DS18B20 temperature sensor
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * void start_Py_ds18b20(void)
 *
 * Writes temperature values from the DS18B20 temperature sensor into
 * temp.fifo
***************************************************************************/
void start_Py_ds18b20(void)
{
  std::FILE* fd = fopen("read_temp.py", "r");
	PyRun_SimpleFile(fd,"read_temp.py");
	return;
}

/***************************************************************************
 * ds18b20_t ds18b20_read(void)
 *
 * Reads temperature values from temp.fifo
***************************************************************************/
ds18b20_t ds18b20_read(void)
{
	// Create struct to hold temperature data
	ds18b20_t ds18b20;

	// Read temperature values from temp.fifo
	char buf[1000];
    std::FILE *fd = fopen("temp.fifo", "r");
	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f",&ds18b20.temperature);

	// Return a temperature value
	return ds18b20;
}
