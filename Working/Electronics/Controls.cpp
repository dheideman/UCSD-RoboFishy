/******************************************************************************
* Controls.cpp
*
* Contains the yaw_controller() and depth_controller functions
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
* float yaw_controller()
*
* Takes in readings from the IMU and returns a value between -1 and 1 (-100% - 
* +100%) that the port and starboard thrusters should run at
******************************************************************************/

float yaw_controller(bno055_t bno055, pid_data_t yaw_pid)
{
	float motor_percent;
	float DT = 0.005;
	float YAW_SAT = 0.2;

	// control output //
	if( bno055.yaw < 180 ) // AUV is pointed right
	{
		// motor_percent is negative

		motor_percent = yaw_pid.kp*(yaw_pid.err) 
			+ yaw_pid.kd*(bno055.r)+ yaw_pid.ki*yaw_pid.i_err; // yaw controller

	}
	else		// AUV is pointed left
	{
		// motor_percent is positive
	motor_percent = yaw_pid.kp*(yaw_pid.err) + yaw_pid.kd*(bno055.r) 
			+ yaw_pid.ki*yaw_pid.i_err; // yaw controller
	}

	// saturate yaw controller //
	if( motor_percent > YAW_SAT )
	{
		motor_percent=YAW_SAT;
	}
	else if( motor_percent < -YAW_SAT )
	{
		motor_percent = -YAW_SAT;
	}

	yaw_pid.i_err += yaw_pid.err*DT;

	// set current yaw to be the old yaw //
	yaw_pid.old = bno055.yaw;

	return motor_percent;
}



/******************************************************************************
* float depth_controller(float range)
*
* Takes a range-from-bottom reading from the laser range-finder code and 
* returns a value between -1 and 1 (-100% - +100%) that the vertical thruster
* should run at
******************************************************************************/

float depth_controller(float range)
{
	float vert_percent;			// vertical thruster output in a percentage
    float DEPTH_SAT = 1;
    float DT = 0.005;

    // Range error //
    depth_pid.err = range - depth_pid.setpoint;

	// Accumulated range error for integral control //
	depth_pid.i_err += depth_pid.err;

	if( range > depth_pid.setpoint )
	{
		vert_percent = depth_pid.kp*(depth_pid.err) 
			+ depth_pid.ki*(depth_pid.i_err) 
			+ depth_pid.kd*(depth_pid.err/DT); 
	}
	else 
	{
		// Shut off vertical thruster //
		vert_percent = 0;
	}

	// Saturate depth controller //
	if( vert_percent > DEPTH_SAT )
	{
		vert_percent = DEPTH_SAT;
	}
	else if( vert_percent < -DEPTH_SAT )
	{
		vert_percent = -DEPTH_SAT;
	}

	// Set current depth to be the old depth //
	depth_pid.old = depth_pid.current;

	return vert_percent;
}
