/******************************************************************************
* Controller_Functions.cpp
*
* Contains the marchPID() and depth_controller functions
******************************************************************************/
#include "Mapper.h"

/******************************************************************************
* float marchPID()
*
* Takes in readings from the IMU and returns a value between -1 and 1 (-100% -
* +100%) that the port and starboard thrusters should run at
******************************************************************************/

float marchPID(pid_data_t PID, float input)
{
	// Calculating errors
	PID.perr = input - PID.setpoint;
	PID.derr = (input - PID.old)/(PID.dt);
	PID.ierr += PID.dt * (input - PID.setpoint);

	if(PID.ierr > PID.isat)
	{
		PID.ierr = PID.isat;
	}

	if(PID.ierr < -PID.isat)
	{
		PID.ierr = - PID.isat;
	}

	PID.output =   PID.kp * PID.perr
						+ PID.ki * PID.ierr
						+ PID.kd * PID.derr;

	if(PID.output > PID.sat)
	{
		PID.output = PID.sat;
	}


	if(PID.output < -PID.sat)
	{
		PID.output = -PID.sat;
	}
	// set current input to be the old input //
	PID.old = input;

	return PID.output;
}


/******************************************************************************
 * int initialize_motors(int channels[3], float freq)
 *
 * Initializes the PWM board and the ESCs that run the motors
******************************************************************************/
int initialize_motors(int channels[3], float freq)
{
	// setup PCA9685 PWM board //
	int fd = pca9685Setup(PIN_BASE, PCA9685_ADDR, HERTZ);
	if (fd < 0)
	{
		printf("Error in setup\n");
		return fd;
	}

	// reset all PCA9685 PWM board output //
	pca9685PWMReset(fd);

	// set motor outputs to 0 to initialize ESCs //
	for( int i = 0; i < 3; i++ )
	{
		set_motor(i, 0);
	}
	return fd;
}

/******************************************************************************
 * int set_motor(int motornum, float percent)
 *
 * Takes in a value from -1 to 1 (-100 to +100%) and sets the motor
 * outputs accordingly
******************************************************************************/
int set_motor(int motornum, float percent)
{
  // Define characteristics of PWM pulse, microseconds
	float amplitude  = PWM_HIGH_LIMIT - PWM_ZERO_VALUE;

	// Saturation limits
  if( percent >  1.0) percent =  1.0;
  if( percent < -1.0) percent = -1.0;

  // Deadzone check
  if( (percent < MOTOR_DEADZONE) && (percent > -MOTOR_DEADZONE) ) percent = 0.0;

	// Calculate corresponding pwm output value
	int motoroutput = percent * amplitude + PWM_ZERO_VALUE;

	// Spin those motors
  pwmWrite(motornum + PIN_BASE, motoroutput);

  #ifdef DEBUG
  // Print what we told the motor to spin at.
  printf("Set motor %d to %d \n", motornum, motoroutput);
  #endif

	return 1;
}
