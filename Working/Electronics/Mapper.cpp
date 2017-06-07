/******************************************************************************
*	Main script for the 2017 RoboFishy Scripps AUV
******************************************************************************/

#include "Mapper.h"

// Multithreading //
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// Sampling Values //
#define SAMPLE_RATE 200 // sample rate of main control loop (Hz)
#define DT 0.005		// timestep; make sure this is equal to 1/SAMPLE_RATE!

// Conversion Factors //
#define UNITS_KPA 0.1 // converts pressure from mbar to kPa

/******************************************************************************
* Controller Gains
******************************************************************************/

// Yaw Controller //
#define KP_YAW 0.01
#define KI_YAW 0
#define KD_YAW 1

// Depth Controller //
#define KP_DEPTH 0
#define KI_DEPTH 0
#define KD_DEPTH 0

// Saturation Constants //
#define YAW_SAT 1		// upper limit of yaw controller
#define DEPTH_SAT 1		// upper limit of depth controller
#define INT_SAT 10		// upper limit of integral windup
#define DINT_SAT 10		// upper limit of depth integral windup

// Filter Values //
#define A1 0.3			// for MS5837 pressure sensor
#define A2 0.4			// for MS5837 pressure sensor

// Fluid Densities in kg/m^3 //
#define DENSITY_FRESHWATER 997
#define DENSITY_SALTWATER 1029

// Acceleration Due to Gravity in m/s^2 //
#define GRAVITY 9.81

// Depth Start Value //
#define DEPTH_START 50 // starting depth (mm)

// Depth Threshold Value //
#define DEPTH_STOP 10000	// threshold depth (mm)

// Temperature Threshold Value //
#define TEMP_STOP 25	// deg C

// Stop Timer //
#define STOP_TIME 4		// seconds

// SOS Leak Sensor Pin //
#define SOSPIN 27		// connected to GPIO 27

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
setpoint_t setpoint;

// Holds the system state structure with current system statesystem_state_t sstate;
system_state_t sstate;

// Holds the calibration values for the MS5837 pressure sensor
pressure_calib_t pressure_calib;

// Holds the latest pressure value from the MS5837 pressure sensor
ms5837_t ms5837;

// Create structure for storing IMU data
bno055_t bno055;

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
	// Initialize python interpreter
	Py_Initialize();

	//Set up RasPi GPIO pins through wiringPi
	wiringPiSetupGpio();

	// Check if AUV is initialized correctly //
	if( scripps_auv_init()<0 )
	{
		return -1;
	}
	printf("\nAll components are initializated\n");
	substate.mode = INITIALIZING;
	substate.laserarmed = ARMED;


	// Initialize threads
	sched_param param;
	int policy, maxpriority;

	// Initialize priorities
	pthread_attr_init(&tattrlow);
	pthread_attr_init(&tattrmed);
	pthread_attr_init(&tattrhigh);

	// Get max priority
	pthread_attr_getschedpolicy(&tattrlow, &policy);
	maxpriority = sched_get_priority_max(policy);

	// Extract scheduling parameter
	pthread_attr_getschedparam (&tattrlow, &param);

	// Set up low priority
	param.sched_priority = maxpriority/4;
	pthread_attr_setschedparam (&tattrlow, &param);

	// Set up medium priority
	param.sched_priority = maxpriority/2;
	pthread_attr_setschedparam (&tattrmed, &param);

	// Set up high priority
	param.sched_priority = maxpriority-1;
	pthread_attr_setschedparam (&tattrhigh, &param);

	// Thread handles
	pthread_t navigationThread;
	pthread_t depthThread;
	pthread_t safetyThread;
	pthread_t disarmlaserThread;

	// Create threads using modified attributes
	pthread_create (&navigationThread, &tattrmed, navigation, NULL);
//	pthread_create (&depthThread, &tattrmed, depth_thread, NULL);
	pthread_create (&safetyThread, &tattrlow, safety_thread, NULL);

//	pthread_create (&depthThread, &tattrmed, depth_thread, NULL);
//	pthread_create (&disarmlaserThread, &tattrlow, disarmLaser, NULL);

	// Destroy the thread attributes
	pthread_attr_destroy(&tattrlow);
	pthread_attr_destroy(&tattrmed);
	pthread_attr_destroy(&tattrhigh);

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
	// Initialize pressure sensor //
	pressure_calib = init_ms5837();

	while(substate.mode!=STOPPED)
	{
		// Read pressure sensor by passing calibration structure //
		ms5837 = ms5837_read(pressure_calib);

		// Calculate depth (no idea what the magic numbers are)
		depth = (ms5837.pressure-1013)*10.197-88.8; // units?
		// 1013: ambient pressure (mbar)
		// 10.197*p_mbar = p_mmH20

		usleep(10000);
	}

	pthread_exit(NULL);
}

/******************************************************************************
 * Navigation Thread
 *
 * For yaw control
 *****************************************************************************/

void *navigation(void* arg)
{
	initialize_motors(motor_channels, HERTZ);

	//float output_port;		// port motor output
	//float output_starboard; // starboard motor output

	// initialize Motor Percent to be returned by yaw_controller //
	//float motor_percent;

	// Initialize old imu data //
	yaw_pid.old = 0;

	// Initialize setpoint for yaw_controller //
	yaw_pid.setpoint = 0;

	// Initialize error values to be used in yaw_controller //
	yaw_pid.err = 0;
	yaw_pid.i_err = 0;

	// yaw_controller constant initialization //
	yaw_pid.kp = 0.01;
	yaw_pid.kd = 1;
	yaw_pid.ki = .1;

	// Range-from-bottom setpoint //
	depth_pid.setpoint = 2;	// meters

	// Depth controller constant initialization
	depth_pid.kp = 0.001;
	depth_pid.kd = 0;
	depth_pid.ki = 0;

	// Initialize error values to be used in depth_controller //
	depth_pid.err = 0;
	depth_pid.i_err = 0;

	// Hard set motor speed //
	// pwmWrite(PIN_BASE+motor_channels[1], output_starboard)
	//set_motor(0, -0.2);  // right
	//set_motor(1, 0.2); // left
    //set_motor(2, 0.0);

	while(substate.mode!=STOPPED)
	{
		// read IMU values
		bno055 = bno055_read();

	    if (bno055.yaw < 180) //AUV pointed right
		{
			yaw_pid.err = bno055.yaw - yaw_pid.setpoint;
		}
		else //AUV pointed left
		{
			yaw_pid.err =(bno055.yaw-360) - yaw_pid.setpoint;
		}

		// Write captured values to screen
	    /*printf("\nYaw: %f Roll: %f Pitch: %f p: %f q: %f r: %f Sys: %i Gyro: %i Accel: %i Mag: %i\n ",
			 bno055.yaw, bno055.pitch, bno055.roll,
			 bno055.p, bno055.q, bno055.r,
			 bno055.sys, bno055.gyro, bno055.accel,
			 bno055.mag);*/
	
    

		// Sanity test: Check if yaw control works
		
		//Call yaw controller function
		yaw_controller(bno055, yaw_pid);

		// Set port motor
		//set_motor(0,motor_percent);

		// Set starboard motor
		//set_motor(1, motor_percent);

		
		printf("\nYawPID_err: %f Motor Percent: %f ", yaw_pid.err, motor_percent);
	
		// Sleep for 5 ms //
	    if (bno055.yaw < 180)
		{
		yaw_pid.err = abs(bno055.yaw - yaw_pid.setpoint);
		}
		else
		{
		yaw_pid.err =abs((bno055.yaw-360) - yaw_pid.setpoint);
		}

		// Write captured values to screen
	    /*printf("\nYaw: %f Roll: %f Pitch: %f p: %f q: %f r: %f Sys: %i Gyro: %i Accel: %i Mag: %i\n ",
					 bno055.yaw, bno055.pitch, bno055.roll,
					 bno055.p, bno055.q, bno055.r,
					 bno055.sys, bno055.gyro, bno055.accel,
					 bno055.mag);*/
		//printf("\nYawPID_err: %f Motor Percent: %f ", yaw_pid.err, motor_percent);
    

		// Sanity test: Check if yaw control works
		/*
		// Call yaw controller function
		yaw_controller();

		// Set port motor
		set_motor(0,motor_percent);

		// Set starboard motor
		set_motor(1, motor_percent);


		*/

		// Sleep for 5 ms //
//		usleep(5000);
//	}

	//set_motor(0, 0);
	//set_motor(1, 0);
    //set_motor(2, 0);

//	pthread_exit(NULL);
//}


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

	// Leak detection variables //
	int leakStatePin = digitalRead(SOSPIN);	// read the input pin
	int i;									// loop counting integer

	// Leak detection pins //
	pinMode(SOSPIN, INPUT);					// set SOSPIN as an INPUT
	pinMode(17, OUTPUT);					// set GPIO 17 as an OUTPUT
	digitalWrite(leakStatePin, HIGH);		// set GPIO 17 as HIGH (VCC)

	while( substate.mode != STOPPED )
	{
		/******************************************************************************
		 * Depth Protection
		 *
		 * Shut down AUV if vehicle travels deeper than 10m
		 *****************************************************************************/ 
/*
		// get pressure value //
		pressure_calib = init_ms5837();
		ms5837 = ms5837_read(pressure_calib);
		sstate.depth[0] = (ms5837.pressure-1013)*10.197-88.8;
		//sstate.depth[0] = (ms5837.pressure*UNITS_KPA-101.325)/
		//	(DENSITY_FRESHWATER*GRAVITY)*0.000001;		// current depth in mm
		printf("%f\n", sstate.depth[0]);

		// filtered depth value //
		sstate.fdepth[0] = A1*(sstate.depth[0]+sstate.depth[1])+A2*sstate.fdepth[1];
		printf("Current depth is %f\n m", sstate.fdepth[0]/1000);

		//if( sstate.fdepth[0]< DEPTH_STOP )
		if( sstate.depth[0] < DEPTH_STOP )
		{
			substate.mode = RUNNING;
		}
		else
		{
			substate.mode = STOPPED;
			printf("%s\n", "STOPPED");
		}

		// set current depth values as old values //
		sstate.depth[1] = sstate.depth[0];
		sstate.fdepth[1] = sstate.fdepth[0];

		// sleep for 50 ms //
		usleep(50000);
*/
		/******************************************************************************
		 * Temperature Protection
		 *
		 * Shut down AUV if housing temperature exceeds 50 deg C
		 *****************************************************************************/

		// read temperature values from DS18B20 temperature sensor //
		ds18b20 = ds18b20_read(); 	// temperature in deg C

		// let 'em know how hot we are //
		printf("Temperature: %f", ds18b20.temperature);

		// Shut down AUV if housing temperature gets too high //
		if( ds18b20.temperature > TEMP_STOP )
		{
			substate.mode = STOPPED;

			// let 'em' know it's bad //
			printf("It's too hot! Shutting down...\n");
		}

		/******************************************************************************
		 * Leak Protection
		 *
		 * Shut down AUV if a leak is detected
		 *****************************************************************************/
/*
		// check leak sensor for water intrusion //
		if( leakStatePin == HIGH )
		{
			// tell 'em it's bad //
			substate.mode = STOPPED;
			printf("LEAK DETECTED! Shutting down...\n");
        }
		else if (leakStatePin == LOW)
		{
			// let 'em know we're dry //
			printf("We're dry. We're still good to go.\n");
		}
*/
		/******************************************************************************
		 * Collision Protection
		 *
		 * Shut down AUV if a collision is detected
		 *****************************************************************************/
/*
		// check IMU accelerometer for collision (1+ g detected) //
		if( bno055.x_acc > 1.0*GRAVITY || bno055.y_acc > 1.0*GRAVITY 
			|| bno055.z_acc > 1.0*GRAVITY )
		{
			for( i=0; i<3; i++ )
			{
				// shut off all motors //
				set_motor(i, MOTOR_0);
			}

			// let 'em' know we're turning off //
			printf("Collision detected. Shutting down...");
		}

		pthread_exit(NULL);
	}*/

    return NULL;
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
