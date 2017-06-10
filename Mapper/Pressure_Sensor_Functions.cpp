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
  // Create flag for continuing
  bool success = false;
  while (!success)
  {
    // clear fifo file //
    std::FILE* fd = fopen("pressure.fifo","w");
    fclose(fd);
    printf("\nCreated and cleared pressure.fifo\n");

    printf("Starting read_pressure.py, then waiting 3 seconds\n");
  	char cmd[100];
    strcpy(cmd, "nohup python -u read_pressure.py > read_pressure.log 2>&1 < /dev/null & exit");
    system(cmd);
    printf("Started read_pressure.py using system(cmd)\n");
    auv_usleep(3000000);

    // Check whether the python script wrote anything
    std::ifstream pFile("pressure.fifo");
    if(pFile.peek() == std::ifstream::traits_type::eof())
    {
      printf("\nERROR: -----> read_pressure.py has NOT written to pressure.fifo\n");
      printf("Retrying Pressure Sensor initialization\n");
      // if it isn't printing values, restart initialization
    }
    else
    {
      printf("read_pressure.py HAS written to pressure.fifo\n");
      // success! continue
      printf("Pressure Sensor Initialized\n");
      success = true;
    }
  }
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
