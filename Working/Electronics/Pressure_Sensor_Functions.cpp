/******************************************************************************
* MS5837.cpp
*
* File to run Initialization and reading files on the MS5837 Pressure Sensor
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
 * void start_read_pressure
 *
 * Starts pressure reading python program
******************************************************************************/

void start_read_pressure(void)
{
	Py_Initialize();
	FILE* fd = fopen("python read_pressure.py", "r");
	PyRun_SimpleFile(fd,"python read_pressure.py");
	Py_Finalize();
	return;
}

/******************************************************************************
 * ms5837_t read_pressure_fifo
 *
 * Reads depth and temp values from pressure.fifo and write to the ms5837 struct
******************************************************************************/
ms5837_t read_pressure_fifo(void)
{
	ms5837_t ms5837;
	char buf[1000];
	FILE *fd = fopen( "pressure.fifo", "r");
	fgets(buf, 1000, fd);
	fclose(fd);
	sscanf(buf, "%f %f", &ms5837.depth, &ms5837.water_temp);
	return ms5837;
}
