/******************************************************************************
* Temperature_Sensor_Functions.cpp
*
* Runs initialization and reads files on the DS18B20 temperature sensor
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * void start_read_temp(void)
 *
 * Writes temperature values from the DS18B20 temperature sensor into
 * temp.fifo
***************************************************************************/
void start_read_temp(void)
{
  std::FILE* fd = fopen("read_temp.py", "r");
	PyRun_SimpleFile(fd,"read_temp.py");
  return;
}

/***************************************************************************
 * float read_temp_fifo(void)
 *
 * Reads temperature values from temp.fifo
***************************************************************************/
float read_temp_fifo(void)
{
	// Create struct to hold temperature data
	float ds18b20;

	// Read temperature values from temp.fifo
	char buf[1000];
    std::FILE *fd = fopen("temp.fifo", "r");
	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f",&ds18b20);

	// Return a temperature value
	return ds18b20;
}
