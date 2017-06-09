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
  // Create flag for continuing
  bool success = false;
  while (!success)
  {
    // clear fifo file //
    std::FILE* fd = fopen("imu.fifo","w");
    fclose(fd);
    printf("Created and cleared imu.fifo\n");

    printf("Starting read_imu.py, then waiting 2 seconds\n");
    char cmd[50];
    strcpy(cmd,"python read_imu.py & exit");
    system(cmd);
    printf("Started IMU using system(cmd)\n");
    // Wait 2 seconds to let the python script start up
    auv_usleep(2000000);

    // Check whether the python script wrote anything
    std::ifstream pFile("imu.fifo");
    if(pFile.peek() == std::ifstream::traits_type::eof())
    {
      printf("read_imu.py has NOT written to imu.fifo\n");
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
				 &imu.yaw,&imu.roll,&imu.pitch,
				 &imu.q, &imu.p, &imu.r,
				 &imu.sys,&imu.gyro,&imu.accel,
				 &imu.mag,imu.x_acc,&imu.y_acc,&imu.z_acc);

	return imu;
}
