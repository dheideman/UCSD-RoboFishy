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
float yaw_controller(bno055, yaw_pid)
{
	// control output //
	if( bno055.yaw < 180 ) // AUV is pointed right
	{
		// u[2] is negative
		motor_percent = yaw_pid.kp*(yaw_pid.err) 
			+ yaw_pid.kd*(bno055.r)+ yaw_pid.ki*yaw_pid.i_err; // yaw controller
	}
	else		// AUV is pointed left
	{
		// u[2] is positive
		motor_percent = yaw_pid.kp*(yaw_pid.setpoint-(bno055.yaw-360)) 
			+ yaw_pid.kd*(bno055.r) + yaw_pid.ki*yaw_pid.i_err; // yaw controller
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

	return motor_percent;
}
*/


/******************************************************************************
* float depth_controller(float range)
*
* Takes a range-from-bottom reading from the laser range-finder code and 
* regulates the range-from-bottom of the AUV
******************************************************************************/
/*
float depth_controller(float range)
{
	float vert_percent;		// vertical thruster output in a percentage


	if( range > distance )
	{
		vert_percent = depth_pid.kp*(range-depth_pid.setpoint) 
			+ depth_pid.ki*() + depth_pid.kd*((range_current-range_old)/DT); 
	}
	else if ( range < distance )
	{
		// shut off vertical thruster //
		vert_percent = 0;
	}
	else
	{

	}
}*/