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
  
	// Run main while loop, wait until it's time to stop
	while(!stoploop)
	{
		int motornum = 0, motoroutput;
	
		//std::cout << "Input motor number: ";
		//std::cin  >> motornum;
	
		std::cout << "Input motor output: ";
		std::cin  >> motoroutput;
		
		// Spin those motors
		//pwmWrite(motornum + PIN_BASE, motoroutput);
		pwmWrite(PIN_BASE + 0, motoroutput);
		pwmWrite(PIN_BASE + 1, motoroutput);
		
		std::cout << "Set Motor " << motornum << " to " << motoroutput << std::endl << std::endl;
	}

	// Shut off motors
	pwmWrite(PIN_BASE + 0, 2647);
	pwmWrite(PIN_BASE + 1, 2647);
	pwmWrite(PIN_BASE + 2, 2647);

	return 0;
}

