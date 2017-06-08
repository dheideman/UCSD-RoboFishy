/******************************************************************************
* IMU_Functions.cpp
*
* Runs initialization and reads files on the BNO055 IMU
******************************************************************************/
#include "Mapper.h"


/***************************************************************************
 * void start_read_imu
 *
 * Starts read_imu.py code
***************************************************************************/
void start_read_imu(void)
{
  // clear fifo file //
  std::FILE* fifo = fopen("imu.fifo","w");
  fclose(fifo);

  // Start up the Python
  std::FILE* fd = fopen("read_imu.py", "r");
  PyRun_SimpleFile(fd,"read_imu.py");

  // Wait 2 seconds to let the python script start up
  usleep(2000000);

  // Check whether the python script wrote anything
  fifo = fopen("imu.fifo","r");

  // insert check here //

  // if it isn't printing values, restart initialization

  fclose(fifo);

  return;
}

/***************************************************************************
 * bno055_t read_imu_fifo
 *
 * Reads IMU values from imu.fifo
***************************************************************************/
bno055_t read_imu_fifo(void)
{
	bno055_t imu;
	char buf[1000];
	FILE *fd = fopen( "imu.fifo", "r");

	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f %f %f %f %f %f %i %i %i %i",
				 &imu.yaw,&imu.roll,&imu.pitch,
				 &imu.q, &imu.p, &imu.r,
				 &imu.sys,&imu.gyro,&imu.accel,
				 &imu.mag);

	return imu;
}
