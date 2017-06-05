/******************************************************************************
*PWM_Driver_Functions.cpp
*
*File to run Initialization and reading files on the BNO055 IMU
******************************************************************************/
#include "Mapper.h"



/***************************************************************************
 * int initialize_motors
 *
 * Description
***************************************************************************/

// initialize motors function //
int initialize_motors(int channels[4], float freq)
{
	int i;
	int fd = pca9685Setup(PIN_BASE, PCA9685_ADDR, HERTZ); // setup PCA9685 board
	if (fd < 0)
	{
		printf("Error in setup\n");
		return fd;
	}
	pca9685PWMReset(fd);												// reset all output
	//usleep(10);

	// imported from motor.c test code //
	int active=1;
		while (active)
		{
			for( i = 0; i < 3; i++ )
			{
				//pwmWrite (PIN_BASE+i, calcTicks(0,HERTZ));
				pwmWrite (PIN_BASE+i, 2674);	//send input signal that is low enough to reach the
												//"neutral" or power-off area in order to arm the ESC (long beep); green LED on ESC will light up
				delay(1000);
				active=0;
			}
		}
	//usleep(100000);
	return fd;
}