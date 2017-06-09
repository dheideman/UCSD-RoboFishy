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
#define DT 0.005		// timestep; make sure this is equal to 1/SAMPLE_RATE!

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

void *navigation_thread(void* arg);
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

// Holds the latest temperature value from the temperature temperature sensor
float temperature;

// Holds the constants and latest errors of the yaw pid controller
pid_data_t yaw_pid;

// Holds the constants and latest errors of the depth pid controller
pid_data_t depth_pid;

// Motor channels
int motor_channels[] = {CHANNEL_1, CHANNEL_2, CHANNEL_3};

// Ignoring sstate
float depth = 0;

 // setmotor intialization
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
	//pthread_t safetyThread;
	//pthread_t disarmlaserThread;


	// Create threads using modified attributes
	//pthread_create (&disarmlaserThread, &tattrlow, disarmLaser, NULL);
	//pthread_create (&safetyThread, &tattrlow, safety_thread, NULL);
	pthread_create (&depthThread, &tattrmed, depth_thread, NULL);
	//pthread_create (&navigationThread, &tattrmed, navigation_thread, NULL);

  // Destroy the thread attributes
 	destroyTAttr();

  printf("Threads started\n");
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
	printf("Depth Thread Started\n");
	// Initialize pressure sensor
	//pressure_calib = init_pressure_sensor();

	while(substate.mode!=STOPPED)
	{
		printf("In the depth thread while loop\n");
		// Read pressure sensor by passing calibration structure
		//ms5837 = read_pressure();

		//printf("Current Depth:\t %.3f\n", ms5837.depth);
		//usleep(1000000);

		// read IMU values from fifo file
		substate.imu = read_imu_fifo();

		// Write IMU data
		printf("\nYaw: %f Roll: %f Pitch: %f p: %f q: %f r: %f Sys: %i Gyro: "
			"%i Accel: %i Mag: %i X_acc: %f Y_acc: %f Z_acc: %f\n ",
			 substate.imu.yaw, substate.imu.pitch, substate.imu.roll,
			 substate.imu.p, substate.imu.q, substate.imu.r,
			 substate.imu.sys, substate.imu.gyro, substate.imu.accel,
			 substate.imu.mag, substate.imu.x_acc, substate.imu.y_acc,
			 substate.imu.z_acc);

		sleep(1);
	 	printf("\nYawPID_perr: %f Motor Percent: %f ", yaw_pid.perr, motor_percent);
	}

	pthread_exit(NULL);
}//*/

/******************************************************************************
 * Navigation Thread
 *
 * For yaw control
 *****************************************************************************/
/*void *navigation_thread(void* arg)
{
	printf("Nav Thread Started\n");

	initialize_motors(motor_channels, HERTZ);

	float yaw = 0; 			  //Local variable for if statements

  ////////////////////////////////
  // Yaw Control Initialization //
  ////////////////////////////////
	yaw_pid.old = 0;	    	// Initialize old imu data
	yaw_pid.setpoint = 0;   // Initialize setpoint

	yaw_pid.derr = 0;
	yaw_pid.ierr = 0;	    	// Initialize error values
	yaw_pid.perr = 0;

	yaw_pid.kp = KP_YAW;
	yaw_pid.kd = KD_YAW;		// Initialize gain values
	yaw_pid.ki = KI_YAW;

	yaw_pid.isat = INT_SAT;	// Initialize saturation values
	yaw_pid.sat  = YAW_SAT;

	yaw_pid.dt   = DT;      // initialize time step

  //////////////////////////////////
  // Depth Control Initialization //
  //////////////////////////////////
	depth_pid.setpoint = 2; 	// Range-from-bottom setpoint (meters)
	depth_pid.old	   = 0; 		// Initialize old depth
	depth_pid.dt 	   = DT;		// Initialize depth controller time step

	depth_pid.kp = KP_DEPTH;
	depth_pid.kd = KD_DEPTH;	// Depth controller gain initialization
	depth_pid.ki = KI_DEPTH;

	depth_pid.perr = 0;
	depth_pid.ierr = 0;	    	// Initialize depth controller error values
	depth_pid.derr = 0;

	depth_pid.isat = INT_SAT; 	// Depth controller saturation values
	depth_pid.sat  = DEPTH_SAT;

	while(substate.mode!=STOPPED)
	{
		// read IMU values from fifo file
		substate.imu = read_imu_fifo();

	  if (substate.imu.yaw < 180) // AUV pointed right
		{
			yaw = substate.imu.yaw;
		}
		else // AUV pointed left
		{
			yaw =(substate.imu.yaw-360);
		}

		//calculate yaw controller output
		motor_percent = marchPID(yaw_pid, yaw);

		// Set port motor
		set_motor(0, motor_percent);

		// Set starboard motor
		set_motor(1, motor_percent);

		// Sleep for 5 ms
		usleep(5000);
	}

	// Turn motors off
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
/*void *safety_thread(void* arg)
{
	printf("Safety Thread Started\n");

	// Set up WiringPi for use // (not sure if actually needed)
	wiringPiSetup();

	// Leak detection pins
	pinMode(LEAKPIN, INPUT);					// set LEAKPIN as an INPUT
	pinMode(LEAKPOWERPIN, OUTPUT);		// set as output to provide Vcc
	digitalWrite(LEAKPOWERPIN, HIGH);	// write high to provide Vcc

	// Leak checking variables
	int leakState;	// holds the state (HIGH or LOW) of the LEAKPIN

	// Test if temp sensor reads anything
	temperature = read_temp_fifo();
	printf("Temperature: %f degC\n", temperature);

	while( substate.mode != STOPPED )
	{
		// Check if depth threshold has been exceeded
		if( substate.fdepth > DEPTH_STOP )
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

		if( temperature > TEMP_STOP )
		{
			substate.mode = STOPPED;
			printf("It's too hot! Shutting down...\n");
			continue;
		}
		else
		{
			// We're still good
			substate.mode = RUNNING;
		}


		// Check for leak
		leakState = digitalRead(LEAKPIN);	// check the state of LEAKPIN
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
		}

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

}//*/


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
