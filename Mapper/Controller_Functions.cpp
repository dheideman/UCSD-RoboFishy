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

float marchPID(pid_data_t controller, float input)
{
	// Calculating errors
	controller.perr = input - controller.setpoint;
	controller.derr = (input - controller.old)/(controller.dt);
	controller.ierr += controller.dt * (input - controller.setpoint);

	if(controller.ierr > controller.isat)
	{
		controller.ierr = controller.isat;
	}

	if(controller.ierr < -controller.isat)
	{
		controller.ierr = - controller.isat;
	}

	controller.output =   controller.kp * controller.perr
						+ controller.ki * controller.ierr
						+ controller.kd * controller.derr;

	if(controller.output > controller.sat)
	{
		controller.output = controller.sat;
	}


	if(controller.output < -controller.sat)
	{
		controller.output = -controller.sat;
	}
	// set current input to be the old input //
	controller.old = input;

	return controller.output;
}


/******************************************************************************
 * int initialize_motors(int channels[3], float freq)
 *
 * Initializes the PWM board and the ESCs that run the motors
******************************************************************************/
int initialize_motors(int channels[3], float freq)
{
	// integer specifying motor number //
	int i;

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
	int active=1;
		while (active)
		{
			for( i = 0; i < 3; i++ )
			{
				// send "neutral" signal to arm ESCs //
				pwmWrite (PIN_BASE+i, MOTOR_0);
				active=0;
			}
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
  printf("Set motor %d to %d \n",motornum, motoroutput);
  #endif

	return 1;
}

/***************************************************************************
 * int saturate_number(float* val, float min, float max)
 *
 * Generic function for saturating values in a function
***************************************************************************/
int saturate_number(float* val, float min, float max)
{
	// if "val" is greater than "max", set "val" to "max" //
	if(*val>max)
	{
		*val = max;
		return 1;
	}

	// if "val" is less than "min", set "val" to "min" //
	else if(*val<min)
	{
		*val = min;
		return 1;
	}
	return 0;
}