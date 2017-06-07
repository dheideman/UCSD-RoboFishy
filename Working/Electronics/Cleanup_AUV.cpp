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
	usleep(500000);

	// Delete fifo file
	remove("bno055_fifo.txt");
	printf("\nbno055_fifo.txt deleted successfully\n");

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
