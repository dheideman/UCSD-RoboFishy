/******************************************************************************
* YawController.cpp
*
* Contains the yaw_controller() function
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
* int yaw_controller()
*
* Takes in readings from IMU and calculates a percentage (-1 to 1)
******************************************************************************/
/*
int yaw_controller()
{
	// control output //
	if(bno055.yaw<180) // AUV is pointed right
	{
		// u[2] is negative
		motor_percent = yaw_pid.kp*(bno055.yaw - yaw_pid.setpoint); //+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
	}
	else		// AUV is pointed left
	{
		// u[2] is positive
		motor_percent = yaw_pid.kp*(yaw_pid.setpoint-(bno055.yaw-360)); //+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
	}
	// saturate yaw controller //
	if(u[2]>YAW_SAT)
	{
		motor_percent=YAW_SAT;
	}
	else if(motor_percent<-YAW_SAT)
	{
		motor_percent=-YAW_SAT;
	}

	//set current yaw to be the old yaw
	yaw_pid.oldyaw=bno055.yaw;

	//Set starboard positive and port negativ
	starboard_percent = motor_percent;
	port_percent = -motor_percent;

	//set current yaw to be the old yaw
	yaw_pid.oldyaw=bno055.yaw;

	return 0;
}
*/
