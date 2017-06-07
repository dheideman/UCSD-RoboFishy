/******************************************************************************
* PWM_Driver_Functions.cpp
*
* Initializes PCA9685 PWM driver board, initializes ESCs, and sets the 
* motor outputs
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
				pwmWrite (PIN_BASE+i, MOTOR_0);	
//				delay(1000);
				active=0;
			}
		}
	return fd;
}

/******************************************************************************
 * int set_motor(int motornum, float percent)
 *
 * Takes in a value from -1 to 1 (-100 to +100%) and sets the motor
 * outputs accordingly
******************************************************************************/
int set_motor(int motornum, float percent)
{
  // Define deadzone (percent)
  float deadzone = 0.05;

  // Define characteristics of PWM pulse, microseconds
//  int pwmlowlimit  = 1940;
	int pwmhighlimit = 3354;
	int pwmzerovalue = 2647;
	float amplitude  = pwmhighlimit - pwmzerovalue;

	// Saturation limits
  if( percent >  1.0) percent =  1.0;
  if( percent < -1.0) percent = -1.0;
  
  // Deadzone check
  if( (percent < deadzone) && (percent > -deadzone) ) percent = 0.0;
	
	// Calculate corresponding pwm output value
	int motoroutput = percent * amplitude;
	
	// Spin those motors
  pwmWrite(motornum + PIN_BASE, motoroutput);	
  
  #ifdef DEBUG
  // Print what we told the motor to spin at.
  printf("Set motor %d to %d \n",motornum, motoroutput);
  #endif

	return 1;
}
