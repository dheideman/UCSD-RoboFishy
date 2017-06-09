/******************************************************************************
* Cleanup_AUV.cpp
*
* Contains the cleanup_auv() function
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * int cleanup_auv()
 *
 * Cleans up the AUV script, shuts down the motors and closes all threads
***************************************************************************/
int cleanup_auv()
{
	// Set state to exiting
	substate.mode = STOPPED;

	// Let 'em know
	printf("\nExiting Cleanly\n");

	// Let final threads clean up
	auv_usleep(500000);

  // kill python scripts
  char imu_cmd[100];
  strcpy(imu_cmd, "ps -ef | grep read_imu.py | grep -v grep | awk '{print $2}' | xargs kill");
  system(imu_cmd);
  auv_usleep(100000);

	char temp_cmd[100];
  strcpy(temp_cmd, "ps -ef | grep read_temp.py | grep -v grep | awk '{print $2}' | xargs kill");
  system(temp_cmd);
  auv_usleep(100000);

	char pres_cmd[100];
  strcpy(pres_cmd, "ps -ef | grep read_pressure.py | grep -v grep | awk '{print $2}' | xargs kill");
  system(pres_cmd);

	// Delete fifo files
	remove("imu.fifo");
	printf("imu.fifo deleted successfully\n");
	remove("temp.fifo");
	printf("temp.fifo deleted successfully\n");
	remove("pressure.fifo");
	printf("pressure.fifo deleted successfully\n");

	// Set all motors to zero
	int channels[]	= {CHANNEL_1, CHANNEL_2, CHANNEL_3};
	for( int i=0; i<3; i++ )
	{
		// Shut off motors
		pwmWrite (PIN_BASE+channels[i], MOTOR_0);

		// Sleep...just cuz
		usleep(10000);
	}
	return 0;
}
