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
#define UNITS_KPA 0.1		// converts pressure from mbar to kPa

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

// Fluid Densities in kg/m^3
#define DENSITY_FRESHWATER 997
#define DENSITY_SALTWATER 1029

// Acceleration Due to Gravity in m/s^2
#define GRAVITY 9.81

// Depth Start Value
#define DEPTH_START 50 // starting depth (mm)

// Stop Timer
#define STOP_TIME 10		// seconds

// Leak Sensor Inpu and Power Pin
#define LEAKPIN 27		// connected to GPIO 27
#define LEAKPOWERPIN 17 // providing Vcc to leak board

/******************************************************************************
 * Declare Threads
******************************************************************************/

void *navigation_thread(void* arg);
void *depth_thread(void* arg);
void *safety_thread(void* arg);
void *userInterface(void* arg);


/******************************************************************************
 * Global Variables
******************************************************************************/

// Holds the setpoint data structure with current setpoints
//setpoint_t setpoint;

// Holds the latest pressure value from the MS5837 pressure sensor
ms5837_t ms5837;

// Holds the latest temperature value from the temperature temperature sensor
float batt_temp;

// Holds the constants and latest errors of the yaw pid controller
pid_data_t yaw_pid;

// Holds the constants and latest errors of the depth pid controller
pid_data_t depth_pid;

// Motor channels
int motor_channels[] = {CHANNEL_1, CHANNEL_2, CHANNEL_3};

// Ignoring sstate
float depth = 0;

// setmotor intialization
float portmotorspeed = 0;
float starmotorspeed = 0;

// Start time for stop timer
time_t start;

/******************************************************************************
* Main Function
******************************************************************************/

int main()
{
	// capture ctrl+c and exit
	signal(SIGINT, ctrl_c);

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
	pthread_t navigationThread;
	pthread_t depthThread;
	pthread_t safetyThread;
	//pthread_t disarmlaserThread;
	pthread_t uiThread;


	// Create threads using modified attributes
	//pthread_create (&disarmlaserThread, &tattrlow, disarmLaser, NULL);
	pthread_create (&safetyThread, &tattrlow, safety_thread, NULL);
	pthread_create (&depthThread, &tattrmed, depth_thread, NULL);
	pthread_create (&navigationThread, &tattrmed, navigation_thread, NULL);
	pthread_create (&uiThread, &tattrmed, userInterface, NULL);

  // Destroy the thread attributes
 	destroyTAttr();

  printf("Threads started\n");

	// Start timer!
	start = time(0);

	// We're ready to run.  Kinda.  Pause first
	substate.mode = PAUSED;

	// Run main while loop, wait until it's time to stop
	while(substate.mode != STOPPED)
	{
		// Check if we've passed the stop time
		if(difftime(time(0),start) > STOP_TIME)
			substate.mode = PAUSED;

		// Sleep a little
		auv_usleep(100000);
	}

	// Exit cleanly
	cleanup_auv();
	return 0;
}

/******************************************************************************
* Depth Thread
*
* For Recording Depth & Determining If AUV is in Water or not
******************************************************************************/

void *depth_thread(void* arg)
{
	printf("Depth Thread Started\n");

	while(substate.mode!=STOPPED)
	{
	  // Read pressure values
		ms5837 = read_pressure_fifo();

		// read IMU values from fifo file
		substate.imu = read_imu_fifo();

    // Only print while RUNNING
    if(substate.mode == RUNNING)
    {
      printf("\nCurrent Depth:\t %.3f m, Current water temp:\t %.3f C\n",
              ms5837.depth, ms5837.water_temp);

      printf("Current battery temp:\t %.2f\n", read_temp_fifo());

      // Write IMU data
      printf("\nYaw: %5.2f Roll: %5.2f Pitch: %5.2f p: %5.2f q: %5.2f r: %5.2f \nSys: %i Gyro: "
        "%i Accel: %i Mag: %i X_acc: %f Y_acc: %f Z_acc: %f\n ",
         substate.imu.yaw,	substate.imu.roll,	substate.imu.pitch,
         substate.imu.p, 		substate.imu.q,			substate.imu.r,
         substate.imu.sys,	substate.imu.gyro,	substate.imu.accel,
         substate.imu.mag,	substate.imu.x_acc,	substate.imu.y_acc,
         substate.imu.z_acc);
    }
		auv_usleep(1000000);
	}
	pthread_exit(NULL);
}//*/

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
  float basespeed = 0.1;
  float err;

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

  //////////////////////////////////
  // Depth Control Initialization //
  //////////////////////////////////
	depth_pid.setpoint	= 0.5	;		// Range-from-bottom setpoint (meters)
	depth_pid.old				= 0;		// Initialize old depth
	depth_pid.dt				= DT;		// Initialize depth controller time step

	depth_pid.kp = KP_DEPTH;
	depth_pid.kd = KD_DEPTH;		// Depth controller gain initialization
	depth_pid.ki = KI_DEPTH;

	depth_pid.perr = 0;
	depth_pid.ierr = 0;					// Initialize depth controller error values
	depth_pid.derr = 0;

	depth_pid.isat = INT_SAT;		// Depth controller saturation values
	depth_pid.sat  = DEPTH_SAT;

	// read IMU values from fifo file
	substate.imu = read_imu_fifo();

	// Set setpoint to current heading
	yaw_pid.setpoint = substate.imu.yaw;

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

		// Only tell motors to run if we are RUNNING
    if( substate.mode == RUNNING)
    {
      // Print yaw
		  printf("Yaw:%5.0f  ", substate.imu.yaw);
      printf("Yaw setpoint:%5.0f  ", yaw_pid.setpoint);

      //calculate yaw controller output
      motorpercent = marchPID(yaw_pid, yaw);
      
      // Set port motor
      portmotorspeed = set_motor(0, basespeed + motorpercent);

      // Set starboard motor
      starmotorspeed = set_motor(1, basespeed - motorpercent);

      // Print motor speeds
      printf("Port Output:%5.2f  ", portmotorspeed);
      printf("Star Output:%5.2f\n", starmotorspeed);

		} // end if RUNNING
		else if( substate.mode == PAUSED)
		{
		  // Stop horizontal motors
		  set_motor(0, 0);
		  set_motor(1, 0);

		  // Wipe integral error
		  yaw_pid.ierr = 0;

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


/******************************************************************************
 * Safety Thread
 *
 * Shuts down AUV if vehicle goes belows 10m, temperature gets too high, or
 * water intrusion is detected
 *****************************************************************************/
void *safety_thread(void* arg)
{
	printf("Safety Thread Started\n");

	// Leak detection pins
	pinMode(LEAKPIN, INPUT);					// set LEAKPIN as an INPUT
	pinMode(LEAKPOWERPIN, OUTPUT);		// set as output to provide Vcc
	digitalWrite(LEAKPOWERPIN, HIGH);	// write high to provide Vcc

	while( substate.mode != STOPPED )
	{
		// Check if depth threshold has been exceeded
		if( ms5837.depth > STOP_DEPTH )
		{
			substate.mode = STOPPED;
			printf("\nWe're too deep! Shutting down...\n");
			continue;
		}

		// Check battery compartment temperature
		if( read_temp_fifo() > STOP_TEMP )
		{
			substate.mode = STOPPED;
			printf("\nIt's too hot! Shutting down...\n");
			continue;
		}

		// Check for leak
		if( digitalRead(LEAKPIN) == HIGH )
		{
			substate.mode = STOPPED;
			printf("\nLEAK DETECTED! Shutting down...\n");
			continue;
		}

		// Check IMU accelerometer for collision (1+ g detected)
		if(  (float)fabs(substate.imu.x_acc) > 1.0*GRAVITY
			|| (float)fabs(substate.imu.y_acc) > 1.0*GRAVITY
			|| (float)fabs(substate.imu.z_acc) > 1.0*GRAVITY )
		{
			substate.mode = STOPPED;
			printf("\nCollision detected. Shutting down...");
			continue;
		}
	}
	// Exit thread
  pthread_exit(NULL);
}//*/

/******************************************************************************
 * User Interface Thread
 * void* userInterface(void* arg)
 *
 * Interfaces with the user, asks for input
 *****************************************************************************/
 void* userInterface(void* arg)
 {
  // Declare local constant variables
  float _kp = 0.01, _ki = 0, _kd = 0;

  // Wait a until everything is initialized before starting
  while(substate.mode == INITIALIZING)
  {
    // Waiting...
    auv_usleep(100000);
  }

  // Prompt user for values continuously until the program exits
  while(substate.mode != STOPPED)
  {
    // Print Kp, Ki, Kd for reference
    std::cout << "Current Values:" << std::endl;
    printf("Kp:\t%f\tKi:\t%f\tKd:\t%f\n",_kp,_ki,_kd);
    std::cout << std::endl;

    // Prompt for kp
    std::cout << "Kp: ";
    std::cin >> _kp;

    // Prompt for ki
    std::cout << "Ki: ";
    std::cin >> _ki;

    // Prompt for kd
    std::cout << "Kd: ";
    std::cin >> _kd;

    // Give a newline
    std::cout << std::endl;

    // Reset gain values to input values
    yaw_pid.kp = _kp;
    yaw_pid.ki = _ki;
    yaw_pid.kd = _kd;

    // Clear errors
    yaw_pid.perr = 0;
    yaw_pid.ierr = 0;
    yaw_pid.derr = 0;

    // Start RUNNING again
    substate.mode = RUNNING;

    // Restart timer!
	  start = time(0);

    // Aaaaaaand, WAIT!
    auv_usleep(12*1000000);
  }

  // Exit thread
  pthread_exit(NULL);
 }
