/******************************************************************************
*	Motor Calibration tool:  Set motor PWM pulse widths to any value in us
******************************************************************************/

#include "MotorCalibration.h"


/******************************************************************************
* Main Function
******************************************************************************/

int main()
{
  wiringPiSetupGpio();

	int fd = pca9685Setup(PIN_BASE, PCA9685_ADDR, HERTZ);
  pca9685PWMReset(fd);
  for( int i = 0; i < 3; i++ )
  {
    pwmWrite (PIN_BASE+i, 2674);
    printf("Initialized motor at pin_base: %i\n", PIN_BASE+i);
  }
  
  // Run main while loop, wait until it's time to stop
	while(1)
	{
    int motornum = 0, motoroutput;

//    std::cout << "Input motor number: ";
//    std::cin  >> motornum;
    
    std::cout << "Input motor output: ";
    std::cin  >> motoroutput;

    // Spin those motors
    pwmWrite(motornum + PIN_BASE, motoroutput);

    std::cout << "Set Motor " << motornum << " to " << motoroutput << std::endl << std::endl;
	}

	return 0;
}
