/******************************************************************************
* IMU_Functions.cpp
*
* Runs initialization and reads files on the BNO055 IMU
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * void start_Py_bno055
 *
 * Starts bno055_read.py code
***************************************************************************/
void start_Py_bno055(void)
{
	// clear fifo file //
	std::FILE* fd = fopen("bno055_read.py", "r");
	PyRun_SimpleFile(fd,"bno055_read.py");
	usleep(100000);
	std::FILE* fifo = fopen("imu.fifo","r");
	fclose(fifo);

	// check if fifo file has numbers in it //
	return;
}

/***************************************************************************
 * bno055_t bno055_read
 *
 * Reads IMU values from bno055_fifo.txt
***************************************************************************/
bno055_t bno055_read(void)
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
