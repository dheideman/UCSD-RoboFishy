/******************************************************************************
 * initialize_sensors.cpp
 *
 * Contains the scripps_auv_init() function
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
 * int scripps_auv_init(void)
 *
 * Initializes the IMU, pressure sensor, and temperature sensor
******************************************************************************/

int initialize_sensors(void)
{
	printf("Initializing sensors...\n");
	start_read_imu();					// start IMU
	printf("IMU data is being read\n\n");
	auv_usleep(100000);

	start_read_pressure();		// start pressure sensor
	printf("Pressure is being read\n\n");
	auv_usleep(100000);

	start_read_temp();				// start temperature sensor
	printf("Temp is being read\n\n");
	auv_usleep(100000);

	return 0;
}

///////////////////////////////////////////////////////////////////////////////
//															IMU Functions
///////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * void start_read_imu
 *
 * Starts read_imu.py code
******************************************************************************/
void start_read_imu(void)
{
  // Create flag for continuing
  bool success = false;
  while (!success && (substate.mode!=STOPPED))
  {
    // clear fifo file //
    std::FILE* fd = fopen("imu.fifo","w");
    fclose(fd);
    printf("\nCreated and cleared imu.fifo\n");

    printf("Starting read_imu.py, then waiting 3 seconds\n");
    char cmd[100];
    strcpy(cmd,"nohup python -u read_imu.py > read_imu.log 2>&1 < /dev/null & exit");
    system(cmd);
    printf("Started IMU using system(cmd)\n");
    // Wait 2 seconds to let the python script start up
    auv_usleep(3000000);

    // Check whether the python script wrote anything
    std::ifstream pFile("imu.fifo");
    if(pFile.peek() == std::ifstream::traits_type::eof())
    {
      printf("\nERROR: -----> read_imu.py has NOT written to imu.fifo\n");
      printf("Retrying IMU initialization\n");
      // if it isn't printing values, restart initialization
    }
    else
    {
      printf("read_imu.py HAS written to imu.fifo\n");
      // success! continue
      printf("IMU Initialized\n");
      success = true;
    }
  }
  return;
}

/******************************************************************************
 * imu_t read_imu_fifo
 *
 * Reads IMU values from imu.fifo and write to the imu struct
******************************************************************************/
imu_t read_imu_fifo(void)
{
	imu_t imu;
	char buf[1000];
	FILE *fd = fopen( "imu.fifo", "r");
	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f %f %f %f %f %f %i %i %i %i %f %f %f",
				 &imu.yaw,  &imu.roll,  &imu.pitch,
				 &imu.p,    &imu.q,     &imu.r,
				 &imu.sys,  &imu.gyro,  &imu.accel,
				 &imu.mag,  &imu.x_acc, &imu.y_acc, &imu.z_acc);

	return imu;
}

///////////////////////////////////////////////////////////////////////////////
//															Pressure Functions
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * void start_read_pressure
 *
 * Starts pressure reading python program
******************************************************************************/
void start_read_pressure(void)
{
  // Create flag for continuing
  bool success = false;
  while (!success && (substate.mode!=STOPPED))
  {
    // clear fifo file //
    std::FILE* fd = fopen("pressure.fifo","w");
    fclose(fd);
    printf("\nCreated and cleared pressure.fifo\n");

    printf("Starting read_pressure.py, then waiting 3 seconds\n");
  	char cmd[50];
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

///////////////////////////////////////////////////////////////////////////////
//															Temp Functions
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * void start_read_temp(void)
 *
 * Writes temperature values from the DS18B20 temperature sensor into
 * temp.fifo
******************************************************************************/
void start_read_temp(void)
{
char cmd[100];
  printf("\nStarting read_temp.py, then waiting 2 seconds\n");
  strcpy(cmd, "nohup python -u read_temp.py > read_temp.log 2>&1 < /dev/null & exit");
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
	float batt_temp;

	// Read temperature values from temp.fifo
  std::ifstream pFile;
  pFile.open("temp.fifo");
  pFile >> batt_temp;
  pFile.close();

	// Return a temperature value
	return batt_temp;
}

/******************************************************************************
* float read_cpu_temp(void)
******************************************************************************/
float read_cpu_temp(void)
{
	float cpu_temp;
	std::ifstream pFile;
	pFile.open("/sys/class/thermal/thermal_zone0/temp");
	pFile >> cpu_temp;
	pFile.close();
	return cpu_temp;
}
