/******************************************************************************
 *  Main script for the 2017 RoboFishy Scripps AUV
******************************************************************************/

#include "Libraries.h"
// Multithreading
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

// Pitch Controller //
#define KP_PITCH 0
#define KI_PITCH 0
#define KD_PITCH 0

// Yaw Controller //
#define KP_YAW .01
#define KI_YAW 0
#define KD_YAW 1

// Depth Controller //
#define KP_DEPTH 0
#define KI_DEPTH 0
#define KD_DEPTH 0

// Saturation Constants //
#define PITCH_SAT 10	// upper limit of pitch controller
#define YAW_SAT 1			// upper limit of yaw controller
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

// Depth start value //
#define DEPTH_START -50 //starting depth (mm)

// Stop timer
#define STOP_TIME		4		// seconds


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Declare threads /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void *navigation(void* arg);
void *depth_thread(void* arg);


///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Declare functions ////////////////////////////
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Main Function ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int main()
{
		// Initialize python interpreter
	Py_Initialize();

	//Set up Pi GPIO pins through wiringPi
	wiringPiSetupGpio();

	// Check if AUV is initialized correctly //
	if(scripps_auv_init()<0)
	{
		return -1;
	}
	printf("\nAll components are initializated\n");
	set_state(UNINITIALIZED);


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

	// Create threads using modified attributes
	pthread_create (&navigationThread, &tattrmed, navigation, NULL);
	pthread_create (&depthThread, &tattrmed, depth_thread, NULL);



	// Destroy the thread attributes
	pthread_attr_destroy(&tattrlow);
	pthread_attr_destroy(&tattrmed);
	pthread_attr_destroy(&tattrhigh);

	// Start timer!
	time_t timer = time(NULL);

	// Run main while loop, wait until it's time to stop
	while(get_state()!=EXITING)
	{
		// Check if we've passed the stop time
		if(difftime(timer,time(NULL)) > STOP_TIME) set_state(EXITING);

		// Sleep a little
		usleep(100000);
	}
	cleanup_auv();
	return 0;
}

/******************************************************************************
* Depth Thread
*
* For Recording Depth & Determining If AUV is in Water or Not
******************************************************************************/
void *depth_thread(void* arg){
	// Initialize pressure sensor
	pressure_calib = init_ms5837();

	while(get_state()!=EXITING){
		// Read pressure sensor by passing calibration structure
		ms5837 = ms5837_read(pressure_calib);
		// calculate depth (no idea what's up with the function)
		depth = (ms5837.pressure-1013)*10.197-88.8;

		usleep(10000);
	}
    pthread_exit(NULL);
}

///////////////////////////////////////////////////////////////////////////////
/////////////////// Navigation Thread for Main Control Loop ///////////////////
///////////////////////////////////////////////////////////////////////////////
void *navigation(void* arg)
//PI_THREAD (navigation_thread)
{
static float u[4];	// normalized roll, pitch, yaw, throttle, components
initialize_motors(motor_channels, HERTZ);
//static float new_esc[4];
float output_port;		// port motor output
float output_starboard; // starboard motor output

//Initialize old imu data
float yaw_pid.oldyaw = 0;

//Initialize setpoint for yaw controller
float yaw_pid.setpoint = 0;

//Initialize Error values to be used in yaw controller
float yaw_pid.p_err = 0;
float yaw_pid.i_err = 0;
float yaw_pid.d_err = 0;

//Yaw Controller Constant Initialization
float yaw_pid.kp = .01;
float yaw_pid.kd = 1;
float yaw_pid.ki = .1;

//depth controller constant initialization
float depth_pid.kp = .01;
float depth_pid.kd = 1;
float depth_pid.ki = .1;

	while(get_state()!=EXITING)
	{
	// read IMU values
	bno055 = bno055_read();

	// Write captured values to screen
	//printf("\nYaw: %f Roll: %f Pitch: %f p: %f q: %f r: %f Sys: %i Gyro: %i Accel: %i Mag: %i\n ",
	//			 bno055.yaw, bno055.pitch, bno055.roll,
	//			 bno055.p, bno055.q, bno055.r,
	//			 bno055.sys, bno055.gyro, bno055.accel,
	//			 bno055.mag);
	
	// Sanity test: Check if yaw control works

	// setpoints //
	setpoint.yaw = 0;

	// control output //
	if(sstate.yaw[0]<180) // AUV is pointed right
	{
		// u[2] is negative
		u[2] = KP_YAW*(setpoint.yaw-sstate.yaw[0]); //+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
	}
	else		// AUV is pointed left
	{
		// u[2] is positive
		u[2] = KP_YAW*(setpoint.yaw-(sstate.yaw[0]-360)); //+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
	}

	// saturate yaw controller //
	if(u[2]>YAW_SAT)
	{
		u[2]=YAW_SAT;
	}
	else if(u[2]<-YAW_SAT)
	{
		u[2]=-YAW_SAT;
	}

					// mix controls //
					printf("u[2]: %f\n", u[2]);


						output_starboard = output_starboard+0.2*(4905-2718);	// starboard motor max at 40%
						pwmWrite(PIN_BASE+motor_channels[1], output_starboard); //	starboard motor output = base 20% + yaw control output
						pwmWrite(PIN_BASE+motor_channels[0], output_port);				// port motor at base 20%

					// print motor PWM outputs //
					printf("Port PWM Output2: %f Starboard PWM Output2: %f\n",
						output_port, output_starboard);

					delay(250);		// wait 0.25 sec until next read of IMU values

					// sleep for 5 ms //
					//usleep(5000);
				//}
		//}
	}
	pthread_exit(NULL);
}


///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Safety Thread ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/*PI_THREAD (safety_thread)
{
	// read depth values from MS5837 pressure sensor //
	while(get_state()!=EXITING)
	{
		// get pressure value //
		calib = init_ms5837();
		ms5837 = ms5837_read(calib);
		//sstate.depth[0] = (ms5837.pressure-1013)*10.197-88.8;
		sstate.depth[0] = (ms5837.pressure*UNITS_KPA-101.325)/
			(DENSITY_FRESHWATER*GRAVITY)*0.000001;		// current depth in mm
		printf("%f\n", sstate.depth[0]);

		// check if depth is above start threshold //
		//sstate.fdepth[0] = A1*(sstate.depth[0]+sstate.depth[1])+A2*sstate.fdepth[1];
		//if(sstate.fdepth[0]>DEPTH_START)
		if(sstate.depth[0]<DEPTH_START)
		{
			drive_mode = DRIVE_ON;
			printf("%s\n", "DRIVE_ON");
		}
		else
		{
			drive_mode = DRIVE_OFF;
			printf("%s\n", "DRIVE_OFF");
		}

		// set current depth values as old values //
		sstate.depth[1] = sstate.depth[0];
		sstate.fdepth[1] = sstate.fdepth[0];

		//sleep for 50 ms //
		usleep(50000);
	}
	return 0;

	// read temperature values from DS18B20 temperature sensor //
					//ds18b20 = ds18b20_read(); // temperature in deg C
					//printf("Temperature: %f", ds18b20.temperature);
					/*if(ds18b20.temperature>60)
					{
						for( i=0; i<4; i++ )
						{
							pwmWrite(PIN_BASE+i, 2674); // turn motors off if temperature gets too high
						}
					}*/
//}

///////////////////////////////////////////////////////////////////////////////
/////////////////////// Logging Thread for Recording Data /////////////////////
///////////////////////////////////////////////////////////////////////////////
/*
PI_THREAD (logging_thread)
{
	while(get_state()!=EXITING){
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
