#include "Mapper.h"

// state variable for loop and thread control //
enum state_t state = UNINITIALIZED;



/***************************************************************************
 * int saturate_number
 *
 * Description
***************************************************************************/

int saturate_number(float* val, float min, float max)
{
	if(*val>max)
	{
		*val = max;
		return 1;
	}
	else if(*val<min)
	{
		*val = min;
		return 1;
	}
	return 0;
}

/***************************************************************************
 * int scripps_auv_init(void)
 *
 * Initializes the IMU, pressure sensor, and temperature sensor
***************************************************************************/

int scripps_auv_init(void)
{
	start_Py_bno055();			// start IMU
	sleep(10);
	//start_Py_ms5837();		// start pressure sensor
	start_Py_ds18b20();			// start temperature sensor
	signal(SIGINT, ctrl_c);		// capture ctrl+c and exit
	return 0;
}


/***************************************************************************
 * state_t get_state()
 *
 * Gets the AUV's current state
***************************************************************************/

state_t get_state()
{
	return state;
}

/***************************************************************************
 * int set_state(enum state_t new_state)
 *
 * Sets the AUV state
***************************************************************************/

int set_state(enum state_t new_state)
{
	state = new_state;
	return 0;
}

/***************************************************************************
 * void ctrl_c(int signo)
 *
 * Captures a user-entered ctrl+c and exits the script
***************************************************************************/
void ctrl_c(int signo)
{
	if (signo == SIGINT)
	{
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
	}
}

/***************************************************************************
 * int cleanup_auv()
 *
 * Cleans up the AUV script, shuts down the motors and closes all threads
***************************************************************************/
int cleanup_auv()
{
	// Set state to exiting
	set_state(EXITING);

	// let 'em know
	printf("\nExiting Cleanly\n");

	// let final threads clean up
	usleep(500000);

	// delete fifo file
	remove("bno055_fifo.txt")
	printf("\nbno055_fifo.txt deleted successfully\n");

	// Set all motors to zero
	int channels[]	= {CHANNEL_1, CHANNEL_2, CHANNEL_3};
	int i;
	for( i = 0; (i < 3); i = i+1 )
	{
		pwmWrite (PIN_BASE+channels[i], 2674);		// set motor outputs to 0
		// sleep...just cuz
		usleep(10000);
	}
	return 0;
}

/******************************************************************************
* int yaw_controller()
*
* Takes in readings from IMU and calculates a percentage (-1 to 1)
******************************************************************************/
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
/***************************************************************************
 * int set_motors()
 *
 * Takes in a value from -1 to 1 (-100 to +100%) and sets the motor
 * outputs accordingly
***************************************************************************/
int set_motors(int motor_num, float speed)
{
	int motor_num;				// indicates which motor to write to
								// port = 0, starboard = 1, vert = 2
	float motor_output;			// feeds the necessary PWM to the motor
	float per_run = 0.2;		// percentage of full PWM to run at
	//float min_per_run = 0.1;	// minimum percentage of full PWM to run at
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
		pwmWrite(motor_num, motor_output);
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
		motor_output = 2674;	// turn off motor
}
