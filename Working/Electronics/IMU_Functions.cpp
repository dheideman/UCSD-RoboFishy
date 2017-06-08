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
  fclose(fifo);

  return;
}

/***************************************************************************
 * bno055_t read_imu_fifo
 *
 * Reads IMU values from bno055_fifo.txt
***************************************************************************/
bno055_t read_imu_fifo(void)
{
	bno055_t bno055;
	char buf[1000];
	FILE *fd = fopen( "imu.fifo", "r");

	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f %f %f %f %f %f %i %i %i %i",
				 &bno055.yaw,&bno055.roll,&bno055.pitch,
				 &bno055.q, &bno055.p, &bno055.r,
				 &bno055.sys,&bno055.gyro,&bno055.accel,
				 &bno055.mag);

	return bno055;
}
