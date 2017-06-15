/******************************************************************************
*	Motor Calibration tool:  Set motor PWM pulse widths to any value in us
******************************************************************************/

#include "MotorCalibration.h"
#include <signal.h>

//////////////////////
// Global Variables //
//////////////////////

// Stop main loop?
int stoploop = 0;

/******************************************************************************
* void ctrlC(int signo)
*
* Control-C Handler
******************************************************************************/
void ctrlC(int signo)
{
	if (signo == SIGINT)
	{
		stoploop = 1;
		printf("\nReceived SIGINT Ctrl-C\n");

 	  for( int i = 0; i < 3; i++ )
  	{
		  pwmWrite (300+i, 2674);
		  printf("Motor zeroed at pin_base: %i\n", PIN_BASE+i);
	  }
    printf("exited cleanly\n");
	}
}

/******************************************************************************
* Main Function
******************************************************************************/

int main()
{
	// Initialize WiringPi
 	wiringPiSetupGpio();

	// Setup ctrl-c catcher
	signal(SIGINT, ctrlC);

  int fd = pca9685Setup(PIN_BASE, PCA9685_ADDR, HERTZ);
 	pca9685PWMReset(fd);
 	for( int i = 0; i < 3; i++ )
 	{
		pwmWrite (PIN_BASE+i, 2674);
		printf("Initialized motor at pin_base: %i\n", PIN_BASE+i);
	}

	int motornum, motoroutput;

	// Run main while loop, wait until it's time to stop
	while(!stoploop)
	{
		std::cout << "Input motor number: ";
		std::cin  >> motornum;

		std::cout << "Input motor output: ";
		std::cin  >> motoroutput;

		// Spin those motors
		motoroutput = set_motor(motornum, motoroutput);

		std::cout << "Set Motor " << motornum << " to " << motoroutput << std::endl << std::endl;
	}

	return 0;
}

/******************************************************************************
 * float set_motor(int motornum, float percent)
 *
 * Takes in a value from -1 to 1 (-100 to +100%) and sets the motor
 * outputs accordingly
******************************************************************************/
float set_motor(int motornum, float percent)
{
	// Define characteristics of PWM pulse, microseconds
	float amplitude	 = (PWM_HIGH_LIMIT - PWM_ZERO_VALUE)/100;

	// Saturation limits
	if( percent >	 100) percent =	 100;
	if( percent < -100) percent = -100;

	// Deadzone check
	if( (percent < MOTOR_DEADZONE) && (percent > -MOTOR_DEADZONE) ) percent = 0.0;

	// Calculate corresponding pwm output value
	int motoroutput = percent * amplitude + PWM_ZERO_VALUE;

	// Spin those motors
	pwmWrite(motornum + PIN_BASE, motoroutput);

	#ifdef DEBUG
	// Print what we told the motor to spin at.
	printf("Set motor %d to %d \n", motornum, motoroutput);
	#endif

	return percent;
}
