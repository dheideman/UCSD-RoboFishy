/******************************************************************************
* Controls.cpp
*
* Contains the yaw_controller() and depth_controller functions
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
* float yaw_controller()
*
* Takes in readings from IMU and calculates a percentage (-1 to 1) to run 
* motors at
******************************************************************************/
/*
float yaw_controller()
{
	// control output //
	if( bno055.yaw < 180 ) // AUV is pointed right
	{
		// u[2] is negative
		motor_percent = yaw_pid.kp*(bno055.yaw - yaw_pid.setpoint) 
			+ yaw_pid.kd*(bno055.r); // yaw controller
	}
	else		// AUV is pointed left
	{
		// u[2] is positive
		motor_percent = yaw_pid.kp*(yaw_pid.setpoint-(bno055.yaw-360)) 
			+ yaw_pid.kd*(bno055.r); // yaw controller
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

	// set current yaw to be the old yaw //
	yaw_pid.oldyaw = bno055.yaw;

	return motor_percent;
}
*/


/******************************************************************************
* int depth_controller()
*
* Takes a depth reading from the laser range-finder code and regulates the
* depth of the AUV
******************************************************************************/

