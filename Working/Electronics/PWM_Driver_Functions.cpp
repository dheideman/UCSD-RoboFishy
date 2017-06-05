/******************************************************************************
* PWM_Driver_Functions.cpp
*
* Runs initialization and reads files on the PCA9685 PWM board
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
 * int initialize_motors(int channels[3], float freq)
 *
 * Initializes the PWM board and the ESCs that run the motors
******************************************************************************/
int initialize_motors(int channels[3], float freq)
{
	// integer specifying motor number //
	int i;		

	// setup PCA9685 PWM board //
	int fd = pca9685Setup(PIN_BASE, PCA9685_ADDR, HERTZ); 
	if (fd < 0)
	{
		printf("Error in setup\n");
		return fd;
	}

	// reset all PCA9685 PWM board output //
	pca9685PWMReset(fd);	

	// set motor outputs to 0 to initialize ESCs //
	int active=1;
		while (active)
		{
			for( i = 0; i < 3; i++ )
			{
				// send "neutral" signal to arm ESCs //
				pwmWrite (PIN_BASE+i, 2674);	
				delay(1000);
				active=0;
			}
		}
	return fd;
}