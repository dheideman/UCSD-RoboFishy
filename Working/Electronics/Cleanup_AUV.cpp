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
  char cmd[50];
  strcpy(cmd, "ps -ef | grep read_imu.py | grep -v grep | awk '{print $2}' | xargs kill");
  system(cmd);
  auv_usleep(100000);

  strcpy(cmd, "ps -ef | grep read_temp.py | grep -v grep | awk '{print $2}' | xargs kill");
  system(cmd);
  auv_usleep(100000);

  strcpy(cmd, "ps -ef | grep read_pressure.py | grep -v grep | awk '{print $2}' | xargs kill");
  system(cmd);

	// Delete fifo files
	remove("imu.fifo");
	printf("imu.fifo deleted successfully\n");
	remove("temp.fifo");
	printf("temp.fifo deleted successfully\n");
	remove("pressure.fifo");
	printf("pressure.fifo deleted successfully\n");

	// Set all motors to zero
	int channels[]	= {CHANNEL_1, CHANNEL_2, CHANNEL_3};
	int i;
	for( i=0; i<3; i++ )
	{
		// Shut off motors
		pwmWrite (PIN_BASE+channels[i], MOTOR_0);

		// Sleep...just cuz
		usleep(10000);
	}

	// Shutdown Python interpreter
	Py_Finalize();

	return 0;
}
