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

float marchPID(pid_data_t controller, float input)
{	
	//Calculating errors
	controller.kerr = input - controller.setpoint;
	controller.derr = (input - controller.old)/(controller.DT);
	controller.ierr += controller.DT * input;

	if(controller.ierr > controller.isat)
	{
		controller.ierr = controller.isat;
	}

	if(controller.ierr < -controller.isat)
	{
		controller.ierr = - controller.isat;
	}

	controller.output =   controller.kp * controller.kerr
						+ controller.ki * controller.ierr
						+ controller.kd * controller.derr;

	if(controller.output > controller.SAT)
	{
		controller.output = controller.SAT;
	}


	if(controller.output < -controller.SAT)
	{
		controller.output = -controller.SAT;
	}
	// set current input to be the old input //
	controller.old = input;

	return controller.output;
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
