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
/*
	motor_num += PIN_BASE;		// indicates which motor to write to
								// port = 0, starboard = 1, vert = 2
	float motor_output;			// feeds the necessary PWM to the motor
	float motor_max_sat = 1;		// high range of motor output saturation
	float motor_min_sat = 0;		// low range of motor output saturation
	float base_percent = 0.2;	// base percentage of full PWM to run at
	float comp_run = 0.1;		// percentage above base_percent to run	at
  float base_output;
*/
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

	// percent = (-) ----> AUV pointed right (starboard) (range: 2718-4095)
	// percent = (+) ----> AUV pointed left (port) (range: 12-2630)

//	// Calculate motor output //
//	if( percent > 0 )
//	{
//		// set motor output //
//		motor_output = 2630 - percent*port_range;
//
//		// max motor output at 30% //	
//		motor_max_sat = base_output - comp_run*port_range;
//
//		// min motor output at 10% //			
//		motor_min_sat = base_output + comp_run*port_range;
//
//		// saturate motor output at max of 30% //
//		if( motor_output < motor_max_sat )
//		{
//			motor_output = motor_max_sat;
//		}
//		// saturate motor output at min of 10% //
//		else if ( motor_output > motor_min_sat )
//		{
//			motor_output = motor_min_sat;
//		}
//		else
//		{
//			motor_output = 2630-percent*port_range;
//		}
//	}
//    else if( percent < 0 )
//	{
//		// base motor output to 20% //
//		base_output = 2718 - base_percent*starboard_range;
//
//		// set motor output //
//		motor_output = 2718 - percent*starboard_range;	
//
//		// max motor output at 30% //
//		motor_max_sat = base_output + comp_run*starboard_range;
//
//		// min motor output at 10% //		
//		motor_min_sat = base_output - comp_run*starboard_range;
//
//		// saturate motor output at max of 30% //
//		if( motor_output > motor_max_sat )
//		{
//			motor_output = motor_max_sat;
//		}
//		// saturate motor output at min of 10% //
//		else if ( motor_output < motor_min_sat )
//		{
//			motor_output = motor_min_sat;
//		}
//		else
//		{
//			motor_output = 2718 - percent*starboard_range;
//		}
//	}
//	else
//	{
//		motor_output = 2674;	
//	}
//	
//    // Actually tell the motor to spin now
//    pwmWrite(motor_num, motor_output);	
//
//    printf("Set motor %d to %f \n",motor_num, motor_output);
	return 1;
}
