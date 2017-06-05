/******************************************************************************
* Controls.cpp
*
* Contains the yaw_controller() and range_controller functions
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
* int yaw_controller()
*
* Takes in readings from IMU and calculates a percentage (-1 to 1) to run 
* motors at
******************************************************************************/
/*
int yaw_controller()
{
	// control output //
	if( bno055.yaw < 180 ) // AUV is pointed right
	{
		// u[2] is negative
		motor_percent = yaw_pid.kp*(bno055.yaw - yaw_pid.setpoint) 
			+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
	}
	else		// AUV is pointed left
	{
		// u[2] is positive
		motor_percent = yaw_pid.kp*(yaw_pid.setpoint-(bno055.yaw-360)) 
			+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
	}
	// saturate yaw controller //
	if( u[2] > YAW_SAT )
	{
		motor_percent=YAW_SAT;
	}
	else if( motor_percent < -YAW_SAT )
	{
		motor_percent = -YAW_SAT;
	}

	// set current yaw to be the old yaw //
	yaw_pid.oldyaw = bno055.yaw;

	// set starboard positive and port negative //
	starboard_percent = motor_percent;
	port_percent = -motor_percent;

	// set current yaw to be the old yaw //
	yaw_pid.oldyaw = bno055.yaw;

	return 0;
}
*/


/******************************************************************************
* int range_controller()
*
* Takes a range-from-bottom reading from the laser range-finder code and 
* regulates the range-from-bottom of the AUV
******************************************************************************/

int range_controller(float range)
{

}