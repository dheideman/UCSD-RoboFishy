/******************************************************************************
*BNO055.cpp
*
*File to run Initialization and reading files on the BNO055 IMU
******************************************************************************/

#include "Mapper.h"

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// BNO055 FUNCTIONS //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/***************************************************************************
 * void start_Py_bno055
 *
 * Description
***************************************************************************/

void start_Py_bno055(void)	//	start bno055_read.py code
{
	// clear fifo file
	FILE* fd = fopen("bno055_read.py", "r");
	PyRun_SimpleFile(fd,"bno055_read.py");
	//nanosleep(100*1000000);
	FILE* fifo = fopen("bno055_fifo.txt","r");
	fclose(fifo);

	// check if fifo file has numbers in it
	return;
}

/***************************************************************************
 * bno055_t bno055_read
 *
 * Description
***************************************************************************/

bno055_t bno055_read(void)	// read values from bno055 IMU
{
	bno055_t bno055;
	char buf[1000];
	FILE *fd = fopen( "bno055_fifo.txt", "r");
	//FILE *fd = fopen( "/home/pi/UCSD-RoboFishy/Working/Electronics/bno055_fifo.txt", "r");

	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f %f %f %f %f %f %i %i %i %i",
				 &bno055.yaw,&bno055.roll,&bno055.pitch,
				 &bno055.q, &bno055.p, &bno055.r,
				 &bno055.sys,&bno055.gyro,&bno055.accel,
				 &bno055.mag);

	return bno055;
}
