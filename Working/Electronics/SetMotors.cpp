/******************************************************************************
* SetMotors.cpp
*
* Contains the set_motor() function
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * int set_motor()
 *
 * Takes in a value from -1 to 1 (-100 to +100%) and sets the motor
 * outputs accordingly
***************************************************************************/
int set_motor(int motor_num, float speed)
{
	int motor_num;				// indicates which motor to write to
								// port = 0, starboard = 1, vert = 2
	float motor_output;			// feeds the necessary PWM to the motor
	float per_base = 0.2;		// base percentage of full PWM to run at
	float per_run = 0.1;		// percentage above per_base to run at

	int port_range = 2618;		// port motor range
	int starboard_range = 2187; // starboard motor range

	// speed = (-) ----> AUV pointed right (starboard) (range: 2718-4095)
	// speed = (+) ----> AUV pointed left (port) (range: 12-2630)

	// Calculate motor output //
	if( speed < 0 )
	{
		motor_output = 2630 - per_run*port_range;				// set motor output

		// saturate motor output at 20% //de
		if( motor_output < (2630-per_run*port_range) )
		{
			motor_output = (2630-per_run*port_range);
		}
	}
	if( speed > 0 )
	{
		motor_output = 2718 + per_run*starboard_range;			// set motor output

		// saturate motor output at 20% //
		if( motor_output > 2718 + per_run*starboard_range )
		{
			motor_output = 2718 + per_run*starboard_range;
		}
	}
	else
		motor_output = 2674;	// If all else fails, turn off the motor!
	
    // Set the motor speeds
	pwmWrite(motor_num, motor_output);
	
    printf("Set motor %d to %d\n", motor_num, motor_output);

	return 1;
}