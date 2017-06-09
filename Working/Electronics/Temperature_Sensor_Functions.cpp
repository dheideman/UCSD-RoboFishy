/******************************************************************************
* Temperature_Sensor_Functions.cpp
*
* Runs initialization and reads files on the DS18B20 temperature sensor
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
 * void start_read_temp(void)
 *
 * Writes temperature values from the DS18B20 temperature sensor into
 * temp.fifo
******************************************************************************/
void start_read_temp(void)
{
char cmd[50];
  printf("\nStarting read_temp.py, then waiting 2 seconds\n");
  strcpy(cmd, "python read_temp.py & exit");
  system(cmd);
  printf("Started read_temp.py using system(cmd)\n");
  auv_usleep(2000000);
  return;
}

/******************************************************************************
 * float read_temp_fifo(void)
 *
 * Reads temperature values from temp.fifo
******************************************************************************/
float read_temp_fifo(void)
{
	float temp;

	// Read temperature values from temp.fifo
	char buf[100];
  std::FILE *fd = fopen("temp.fifo", "r");
	fgets(buf, 100, fd);
	fclose(fd);
	sscanf(buf, "%f", &temp);

	// Return a temperature value
	return temp;
}
