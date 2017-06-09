/******************************************************************************
* IMU_Functions.cpp
*
* Runs initialization and reads files on the BNO055 IMU
******************************************************************************/
#include "Mapper.h"


/******************************************************************************
 * void start_read_imu
 *
 * Starts read_imu.py code
******************************************************************************/
void start_read_imu(void)
{
  char cmd[50];
  strcpy(cmd,"python read_imu.py & exit");
  system(cmd);
  printf("Started IMU using system(cmd)\n");

  // clear fifo file //
  std::FILE* fd = fopen("imu.fifo","w");
  fclose(fd);
  printf("Cleared imu.fifo\n");

  // Wait 2 seconds to let the python script start up
  usleep(3000000);

  // Check whether the python script wrote anything
  pFile = fopen("imu.fifo","r");
  if(pFile.peek() == std::ifstream::traits_type::eof())
  {
    printf("read_imu.py has NOT writen to imu.fifo");
  }
  else
  {
    printf("read_imu.py has written to imu.fifo");
  }
  fclose(pFile);
/*  fseek(fd, 0, SEEK_END); // go to end of file
  if (ftell(fd) == 0)
  {
    printf("imu.fifo is NOT being written to\n");
  }
  else
  {
    printf("imu.fifo is being written to\n");
  }
  fseek(fd, 0, SEEK_SET); // go to begin of file

  // if it isn't printing values, restart initialization

  fclose(fd);
//*/
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
	sscanf(buf,"%f %f %f %f %f %f %i %i %i %i %f %f %d",
				 &imu.yaw,&imu.roll,&imu.pitch,
				 &imu.q, &imu.p, &imu.r,
				 &imu.sys,&imu.gyro,&imu.accel,
				 &imu.mag,imu.x_acc,&imu.y_acc,&imu.z_acc);

	return imu;
}
