#include "Mapper.h"

/******************************************************************************
 * Navigation Thread
 *
 * For yaw control
 *****************************************************************************/
void *navigation_thread(void* arg)
{
	printf("Nav Thread Started\n");

	initialize_motors(motor_channels, HERTZ);

  float yaw = 0; 			  //Local variable for if statements
  float motorpercent;
  float err;
  float depthpercent;
  float vertmotorspeed;

  ////////////////////////////////
  // Yaw Control Initialization //
  ////////////////////////////////
	yaw_pid.old 			= 0;	 	// Initialize old imu data
	yaw_pid.setpoint 	= 0;		// Initialize setpoint

	yaw_pid.derr = 0;
	yaw_pid.ierr = 0;					// Initialize error values
	yaw_pid.perr = 0;

	yaw_pid.kp = KP_YAW;
	yaw_pid.kd = KD_YAW;			// Initialize gain values
	yaw_pid.ki = KI_YAW;

	yaw_pid.isat = INT_SAT;		// Initialize saturation values
	yaw_pid.sat  = YAW_SAT;

	yaw_pid.dt   = DT;				// initialize time step

	yaw_pid.period = 360;			//set period for yaw controller
  //////////////////////////////////
  // Depth Control Initialization //
  //////////////////////////////////
	depth_pid.setpoint	= 0.5	;		// Range-from-bottom setpoint (meters)
	depth_pid.old				= 0;			// Initialize old depth
	depth_pid.dt				= DT;			// Initialize depth controller time step

	depth_pid.kp = KP_DEPTH;
	depth_pid.kd = KD_DEPTH;			// Depth controller gain initialization
	depth_pid.ki = KI_DEPTH;

	depth_pid.perr = 0;
	depth_pid.ierr = 0;						// Initialize depth controller error values
	depth_pid.derr = 0;

	depth_pid.isat = INT_SAT;			// Depth controller saturation values
	depth_pid.sat  = DEPTH_SAT;
	depth_pid.period = -1; 				//not cyclical controller


	// read IMU values from fifo file
	substate.imu = read_imu_fifo();

	// Read pressure values
	ms5837 = read_pressure_fifo();

	//Set depth setpoint to current depth
	depth_pid.setpoint = ms5837.depth + 0.3;

	while(substate.mode!=STOPPED)
	{
		// read IMU values from fifo file
		substate.imu = read_imu_fifo();
		//read depth from pressure sensor
		ms5837 = read_pressure_fifo();

		// Only tell motors to run if we are RUNNING
    if( substate.mode == RUNNING)
    {
      //calculate yaw controller output
      motorpercent = marchPID(yaw_pid, substate.imu.yaw);

      //calculate depth controller output
      depthpercent = marchPID(depth_pid, ms5837.depth);

      // Set port motor
      portmotorspeed = set_motor(0, BASE_SPEED - motorpercent);

      // Set starboard motor
      starmotorspeed = set_motor(1, BASE_SPEED + motorpercent);

      //set vertical thruster
      vertmotorspeed = set_motor(2, depthpercent);
		} // end if RUNNING
		else if( substate.mode == PAUSED)
		{
		  // Stop horizontal motors
		  set_motor(0, 0);
		  set_motor(1, 0);
		  set_motor(2, 0);

		  // Wipe integral error
		  yaw_pid.ierr = 0;
			depth_pid.ierr = 0;

		  // Sleep a while (we're not doing anything anyways)
		  auv_msleep(100);

		} // end if PAUSED

		// Sleep for 5 ms
		auv_usleep(DT);
	}

	// Turn motors off
	set_motor(0, 0);
	set_motor(1, 0);
	set_motor(2, 0);

	pthread_exit(NULL);
}//*/
