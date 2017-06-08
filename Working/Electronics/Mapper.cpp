/******************************************************************************
*	Main script for the 2017 RoboFishy Scripps AUV
******************************************************************************/

#include "Mapper.h"

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// Sampling Values
#define SAMPLE_RATE 200 // sample rate of main control loop (Hz)
#define DT 0.005				// timestep; make sure this is equal to 1/SAMPLE_RATE!

// Conversion Factors
#define UNITS_KPA 0.1 // converts pressure from mbar to kPa

/******************************************************************************
* Controller Gains
******************************************************************************/

// Yaw Controller
#define KP_YAW 0.01
#define KI_YAW 0
#define KD_YAW 1

// Depth Controller
#define KP_DEPTH 0
#define KI_DEPTH 0
#define KD_DEPTH 0

// Saturation Constants
#define YAW_SAT 1			// upper limit of yaw controller
#define DEPTH_SAT 1		// upper limit of depth controller
#define INT_SAT 10		// upper limit of integral windup
#define DINT_SAT 10		// upper limit of depth integral windup

// Filter Values
#define A1 0.3			// for MS5837 pressure sensor
#define A2 0.4			// for MS5837 pressure sensor

// Fluid Densities in kg/m^3
#define DENSITY_FRESHWATER 997
#define DENSITY_SALTWATER 1029

// Acceleration Due to Gravity in m/s^2
#define GRAVITY 9.81

// Depth Start Value
#define DEPTH_START 50 // starting depth (mm)

// Stop Timer
#define STOP_TIME 4		// seconds

// Leak Sensor Inpu and Power Pin
#define LEAKPIN 27		// connected to GPIO 27
#define LEAKPOWERPIN 17 // providing Vcc to leak board

/******************************************************************************
 * Declare Threads
******************************************************************************/

void *navigation(void* arg);
void *depth_thread(void* arg);
void *safety_thread(void* arg);


/******************************************************************************
 * Global Variables
******************************************************************************/

// Holds the setpoint data structure with current setpoints
//setpoint_t setpoint;

// Holds the calibration values for the MS5837 pressure sensor
pressure_calib_t pressure_calib;

// Holds the latest pressure value from the MS5837 pressure sensor
ms5837_t ms5837;

// Create structure for storing IMU data
//bno055_t bno055;

// Holds the latest temperature value from the DS18B20 temperature sensor
ds18b20_t ds18b20;

// Holds the constants and latest errors of the yaw pid controller
pid_data_t yaw_pid;

// Holds the constants and latest errors of the depth pid controller
pid_data_t depth_pid;

// Motor channels
int motor_channels[] = {CHANNEL_1, CHANNEL_2, CHANNEL_3};

// Ignoring sstate
float depth = 0;

//yaw_controller intialization
float motor_percent = 0;



/******************************************************************************
* Main Function
******************************************************************************/

int main()
{
	// Initialize Python interpreter
	Py_Initialize();

	// Set up RasPi GPIO pins through wiringPi
	wiringPiSetupGpio();

	// Check if AUV is initialized correctly
	if( initialize_sensors() < 0 )
	{
		return -1;
	}
	printf("\nAll components are initialized\n");
	substate.mode = INITIALIZING;
    substate.laserarmed = ARMED;

	printf("Starting Threads\n");
	initializeTAttr();

	// Thread handles
	//pthread_t navigationThread;
	pthread_t depthThread;
	pthread_t safetyThread;
	//pthread_t disarmlaserThread;


	// Create threads using modified attributes
	//pthread_create (&disarmlaserThread, &tattrlow, disarmLaser, NULL);
	pthread_create (&safetyThread, &tattrlow, safety_thread, NULL);
	pthread_create (&depthThread, &tattrmed, depth_thread, NULL);

// Destroy the thread attributes
 	destroyTAttr();

	// Start timer!
	time_t start = time(0);

	// Run main while loop, wait until it's time to stop
	while(substate.mode != STOPPED)
	{
		// Check if we've passed the stop time
		if(difftime(time(0),start) > STOP_TIME)
			substate.mode = STOPPED;

		// Sleep a little
		usleep(100000);
	}

	// Exit cleanly
	cleanup_auv();
	return 0;
}

/******************************************************************************
* Depth Thread
*
* For Recording Depth & Determining If AUV is in Water or Not
******************************************************************************/

void *depth_thread(void* arg)
{
	// Initialize pressure sensor
	pressure_calib = init_pressure_sensor();

	while(substate.mode!=STOPPED)
	{
		// Read pressure sensor by passing calibration structure
		ms5837 = read_pressure(pressure_calib);

		// Calculate depth (no idea what the magic numbers are)
		depth = (ms5837.pressure-1013)*10.197-88.8; // units?
		// 1013: ambient pressure (mbar)
		// 10.197*p_mbar = p_mmH20

		printf("Current Depth:\t %.3f\n",depth);
		usleep(1000000);
	}

	pthread_exit(NULL);
}//*/

/******************************************************************************
 * Navigation Thread
 *
 * For yaw control
 *****************************************************************************/
/*void *navigation(void* arg)
{


	initialize_motors(motor_channels, HERTZ);

	//float output_port;	  // port motor output
	//float output_starboard; // starboard motor output

	float yaw = 0; 			  //Local variable for if statements

	//////////////yaw controller initialization////////////////////////////////
	yaw_pid.old = 0;	    // Initialize old imu data
	yaw_pid.setpoint = 0;   // Initialize setpoint for yaw_controller

	yaw_pid.derr = 0;
	yaw_pid.ierr = 0;	    // Initialize yaw_controller error values
	yaw_pid.kerr = 0;

	yaw_pid.kp = KP_YAW;
	yaw_pid.kd = KD_YAW;	// Initialize yaw_controller gain values
	yaw_pid.ki = KI_YAW;

	yaw_pid.isat = INT_SAT;	// Initialize saturation values
	yaw_pid.SAT  = YAW_SAT;

	yaw_pid.DT   = DY;      // initialize time step

	/////////////depth controller initialization///////////////////////////////
	depth_pid.setpoint = 2; 	// Range-from-bottom setpoint (meters)
	depth_pid.old	   = 0; 	// Initialize old depth
	depth_pid.DT 	   = DT;	// Initialize depth controller time step

	depth_pid.kp = KP_DEPTH;
	depth_pid.kd = KD_DEPTH;	// Depth controller gain initialization
	depth_pid.ki = KI_DEPTH;

	depth_pid.kerr = 0;
	depth_pid.ierr = 0;	    	// Initialize depth controller error values
	depth_pid.derr = 0;

	depth_pid.isat = INT_SAT; 	//Depth controller saturation values
	depth_pid.SAT  = DEPTH_SAT;

	while(substate.mode!=STOPPED)
	{
		// read IMU values from fifo file
		//bno055 = read_imu_fifo();
		substate.imu = read_imu_fifo();

	    if (substate.imu.yaw < 180) //AUV pointed right
		{
			yaw = substate.imu.yaw;
		}
		else //AUV pointed left
		{
			yaw =(substate.imu.yaw-360);
		}

		// Write captured values to screen
	    //printf("\nYaw: %f Roll: %f Pitch: %f p: %f q: %f r: %f Sys: %i Gyro: %i Accel: %i Mag: %i\n ",
			 bno055.yaw, bno055.pitch, bno055.roll,
			 bno055.p, bno055.q, bno055.r,
			 bno055.sys, bno055.gyro, bno055.accel,
			 bno055.mag);*/


		//calculate yaw controller output
/*		motor_percent = marchPID(substate.imu, yaw);

		// Set port motor
		//set_motor(0,motor_percent);

		// Set starboard motor
		//set_motor(1, motor_percent);

		// Sleep for 5 ms //
	  if (substate.imu.yaw < 180)
		{
			yaw_pid.err = abs(substate.imu.yaw - yaw_pid.setpoint);
		}
		else
		{
			yaw_pid.err =abs((substate.imu.yaw - 360) - yaw_pid.setpoint);
		}

		// Write captured values to screen
	  printf("\nYaw: %f Roll: %f Pitch: %f p: %f q: %f r: %f Sys: %i Gyro: %i Accel: %i Mag: %i\n ",
					 bno055.yaw, bno055.pitch, bno055.roll,
					 bno055.p, bno055.q, bno055.r,
					 bno055.sys, bno055.gyro, bno055.accel,
					 bno055.mag);

		//printf("\nYawPID_err: %f Motor Percent: %f ", yaw_pid.err, motor_percent);


		// Call yaw controller function
		yaw_controller();

		// Set port motor
		set_motor(0,motor_percent);

		// Set starboard motor
		set_motor(1, motor_percent);




		// Sleep for 5 ms
		usleep(5000);
	}

	set_motor(0, 0);
	set_motor(1, 0);
	set_motor(2, 0);

	pthread_exit(NULL);
}//*/


/******************************************************************************
 * Safety Thread
 *
 * Shuts down AUV if vehicle goes belows 10m, temperature gets too high, or
 * water intrusion is detected
 *****************************************************************************/
void *safety_thread(void* arg)
{
	// Set up WiringPi for use // (not sure if actually needed)
	wiringPiSetup();

	// Leak detection pins
	pinMode(LEAKPIN, INPUT);					// set LEAKPIN as an INPUT
	pinMode(LEAKPOWERPIN, OUTPUT);		// set as output to provide Vcc
	digitalWrite(LEAKPOWERPIN, HIGH);	// write high to provide Vcc

	// Leak checking variables
	int leakState;	// holds the state (HIGH or LOW) of the LEAKPIN

	// Test if temp sensor reads anything
	ds18b20 = read_temp_fifo();
	printf("Temperature: %f degC\n", ds18b20.temperature);

	while( substate.mode != STOPPED )
	{
		// Check if depth threshold has been exceeded
		/*if( substate.fdepth > DEPTH_STOP )
		{
			substate.mode = STOPPED;
			printf("We're too deep! Shutting down...\n");
			continue;
		}
		else
		{
			// We're still good
			substate.mode = RUNNING;
		}

		// Check temperature
		// Shut down AUV if housing temperature gets too high

		if( ds18b20.temperature > TEMP_STOP )
		{
			substate.mode = STOPPED;
			printf("It's too hot! Shutting down...\n");
			continue;
		}
		else
		{
			// We're still good
			substate.mode = RUNNING;
		}//*/


		// Check for leak
		/*leakState = digitalRead(LEAKPIN);	// check the state of LEAKPIN
		if( leakState == HIGH )
		{
			substate.mode = STOPPED;
			printf("LEAK DETECTED! Shutting down...\n");
			continue;
		}
		else if (leakState == LOW)
		{
			// We're still good
			substate.mode = RUNNING;
		}*/

		// Check IMU accelerometer for collision (1+ g detected)
		if( (float)fabs(substate.imu.x_acc) > 1.0*GRAVITY
			|| (float)fabs(substate.imu.y_acc) > 1.0*GRAVITY
			|| (float)fabs(substate.imu.z_acc) > 1.0*GRAVITY )
		{
			substate.mode = STOPPED;
			printf("Collision detected. Shutting down...");
			continue;
		}
		else
		{
			// We're still good
			substate.mode = RUNNING;
		}

	}
    pthread_exit(NULL);

}


/******************************************************************************
 * Logging Thread
 *
 * Logs the sensor output data into a file
 *****************************************************************************/
/*
PI_THREAD (logging_thread)
{
	while(substate.mode!=STOPPED){
		FILE *fd = fopen("log.txt", "a");
		char buffer[100] = {0};
		// add logging values to the next line
		sprintf(buffer, "%f %f %f %f %i %i %i %i %f %f %f %f\n",sstate.roll, sstate.pitch[0], sstate.yaw[0], sstate.depth[0],sstate.x[0],
		sstate.y[0], sstate.radius[0], setpoint.x - sstate.x[0], sstate.esc_out[0], sstate.esc_out[1], sstate.esc_out[2], sstate.esc_out[3]);
		fputs(buffer, fd);
		fclose(fd);
		//sleep for 100 ms

		usleep(100000);
	}
	return 0;
}
*/
