/******************************************************************************
 * Protection.cpp
 *
 * Contains the depth, temperature and collision protection functions
 *****************************************************************************/ 

#include "Mapper.h"

/******************************************************************************
 * Depth Protection
 *
 * Shut down AUV if vehicle travels deeper than 10m
 *****************************************************************************/ 
/*sub_state_t pressure_protect(float pressure, float fdepth)
{
	// Declare variable to hold the submode
	enum submode;
	
	// Check if depth threshold has been exceeded
	if( fdepth > DEPTH_STOP )
	{
		submode = STOPPED;
		printf("%s\n", "STOPPED");
	}
	else
	{
		// We're still good
		submode = RUNNING;
	}

	// Sleep for 50 ms //
	usleep(50000);

	return submode;
}*/

/******************************************************************************
 * Temperature Protection
 *
 * Shut down AUV if housing temperature exceeds 50 deg C
 *****************************************************************************/
/*sub_state_t temp_protection(float temperature)
{
	// Declare variable to hold the submode
	enum submode;

	// Read temperature values from DS18B20 temperature sensor 
	//ds18b20 = ds18b20_read(); 	// temperature in deg C

	// Let 'em know how hot we are 
	printf("Temperature: %f", temperature);

	// Shut down AUV if housing temperature gets too high 
	if( temperature > TEMP_STOP )
	{
		submode = STOPPED;

		// Let 'em' know it's bad 
		printf("It's too hot! Shutting down...\n");
	}
	else
	{
		// We're still good
		submode = RUNNING;
	}

	return submode;
}*/
/******************************************************************************
 * Leak Protection
 *
 * Shut down AUV if a leak is detected
 *****************************************************************************/
/*sub_state_t leak_protection(int leakState)
{
	// Declare variable to hold the submode
	enum submode;

	// Check leak sensor for water intrusion 
	if( leakState == HIGH )
	{
		// Tell 'em it's bad 
		submode = STOPPED;
		printf("LEAK DETECTED! Shutting down...\n");
	}
	else if (leakState == LOW)
	{
		// We're still good 
		submode = RUNNING;
	}

	return submode;
}
*/
/******************************************************************************
 * Collision Protection
 *
 * Shut down AUV if a collision is detected
 *****************************************************************************/
/*sub_state_t collision_protection(float x_acc, float y_acc, float z_acc)
{
	// Declare variable to hold the submode
	enum submode;

	// Check IMU accelerometer for collision (1+ g detected) 
	if( (float)fabs(x_acc) > 1.0*GRAVITY || (float)fabs(y_acc) > 1.0*GRAVITY 
		|| (float)fabs(z_acc) > 1.0*GRAVITY )
	{
		submode = STOPPED;

		// Let 'em' know we're turning off 
		printf("Collision detected. Shutting down...");
	}
	else
	{
		// We're still good
		submode = RUNNING;
	}

	return submode;
}*/
