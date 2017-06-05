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
	substate.mode = STOPPING;

	// let 'em know
	printf("\nExiting Cleanly\n");

	// let final threads clean up
	usleep(500000);

	// delete fifo file
	remove("bno055_fifo.txt")
	printf("\nbno055_fifo.txt deleted successfully\n");

	// Set all motors to zero
	int channels[]	= {CHANNEL_1, CHANNEL_2, CHANNEL_3};
	int i;
	for( i = 0; (i < 3); i = i+1 )
	{
		pwmWrite (PIN_BASE+channels[i], 2674);		// set motor outputs to 0
		// sleep...just cuz
		usleep(10000);
	}
	return 0;
}
