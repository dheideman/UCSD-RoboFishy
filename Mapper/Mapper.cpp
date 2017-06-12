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

// Print/Logging Rate
#define LOG_RATE      10  // Hz

// Safety Thread Rate
#define SAFETY_RATE   1  // Hz

// Conversion Factors
#define UNITS_KPA 0.1		// converts pressure from mbar to kPa

/******************************************************************************
* Controller Gains
******************************************************************************/

// Yaw Controller
#define KP_YAW 0.0003
#define KI_YAW 0
#define KD_YAW 0

// Depth Controller
#define KP_DEPTH 0.1
#define KI_DEPTH 0
#define KD_DEPTH 0

// Saturation Constants
#define YAW_SAT 1			// upper limit of yaw controller
#define DEPTH_SAT 1		// upper limit of depth controller
#define INT_SAT 10		// upper limit of integral windup
#define DINT_SAT 10		// upper limit of depth integral windup

// Pitch/Roll limits
#define ROLL_LIMIT    1000  // degrees
#define PITCH_LIMIT   1000  // degrees

// Fluid Densities in kg/m^3
#define DENSITY_FRESHWATER 997
#define DENSITY_SALTWATER 1029

// Acceleration Due to Gravity in m/s^2
#define GRAVITY 9.81

// Depth Start Value
#define DEPTH_START 0.05 // starting depth (m)

// Stop Timer
#define STOP_TIME 20		// seconds

// Leak Sensor Inpu and Power Pin
#define LEAKPIN       27	// connected to GPIO 27
#define LEAKPOWERPIN  17  // providing Vcc to leak board

// Time Per Straight Leg of "Path"
#define DRIVE_TIME    6   // seconds


/******************************************************************************
 * Declare Threads
******************************************************************************/

void *navigation_thread(void* arg);
void *log_thread(void* arg);
void *safety_thread(void* arg);
void *userInterface(void* arg);


/******************************************************************************
 * Global Variables
******************************************************************************/

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
struct timeval start, now;

// Setpoint array
float setpoints[] = {0, 90, 180, -90};
int   nsetpoints = 5;

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
	pthread_t logThread;
	pthread_t safetyThread;
	//pthread_t disarmlaserThread;
	pthread_t uiThread;


	// Create threads using modified attributes
	//pthread_create (&disarmlaserThread, &tattrlow, disarmLaser, NULL);
	pthread_create (&safetyThread, &tattrlow, safety_thread, NULL);
	pthread_create (&logThread, &tattrmed, log_thread, NULL);
	pthread_create (&navigationThread, &tattrmed, navigation_thread, NULL);
//	pthread_create (&uiThread, &tattrmed, userInterface, NULL);

  // Destroy the thread attributes
 	destroyTAttr();

  printf("Threads started\n");

	// Start timer!
	gettimeofday(&start, NULL);

	int iterator = 0;

	// We're ready to run.  Kinda.  Pause first
//	substate.mode = PAUSED;
  substate.mode = RUNNING;
	
  // Run main while loop, wait until it's time to stop
	while(substate.mode != STOPPED)
	{
		// Check if we've passed the stop time
		gettimeofday(&now, NULL);
 		if((now.tv_sec - start.tv_sec) > STOP_TIME)
 			substate.mode = STOPPED;
/*
    if(substate.mode == RUNNING)
    {
      // Change the setpoint every DRIVE_TIME seconds
      if(difftime(time(0),start) > DRIVE_TIME)
      {
        // If this was the last segment
        if(iterator >= nsetpoints)
        {
          iterator = 0;
          substate.mode = STOPPED;
        }
        else
        {
          // Reset timer
          start = time(0);

          // Set new setpoint
         // yaw_pid.setpoint = setpoints[iterator];
          // Increment iterator
          iterator++;

        } // end if iterator

      } // end if difftime

    } // end if RUNNING
*/
		// Sleep a little
		auv_usleep(100000);
	}

	// Exit cleanly
	cleanup_auv();
	return 0;
}

/******************************************************************************
* Logging Thread
*
* Writes data to screen and to files
******************************************************************************/

void *log_thread(void* arg)
{
  // Start the Kenny Loggings Thread!
	printf("Logging Thread Started\n");
	
	// open a log to record general data
	std::ofstream genlog;
	genlog.open("mapper.log");
	
	// open a log to record yaw-related information
	std::ofstream yawlog;
	yawlog.open("yawcontrol.log");

	// Create a string stream to store information for printing and logging
	std::stringstream output;

	while(substate.mode!=STOPPED)
	{
	  // Read pressure values
		ms5837 = read_pressure_fifo();

		// read IMU values from fifo file
		substate.imu = read_imu_fifo();

    // Only print while RUNNING
    if(substate.mode == RUNNING)
    {
      /* Generate general output */
      
      // Clear string stream
      output.str(std::string());
      
      // Print yaw
      output << "Yaw: " << substate.imu.yaw << "\t";
      output << "Yaw setpoint: " << yaw_pid.setpoint << std::endl;
      
      // Print depth
      output << "Depth: " << ms5837.depth << "\t";
      output << "Depth setpoint: " << depth_pid.setpoint << "\t";
      output << "Water temp: " << ms5837.water_temp << " C" << std::endl;
      
      // Print battery temperature
      output << "Battery temp: " << read_temp_fifo() << " C" << std::endl;
      
      // Write a newline
      output << std::endl;

      // Write IMU data
/*
      printf("\nYaw: %5.2f Roll: %5.2f Pitch: %5.2f p: %5.2f q: %5.2f r: %5.2f \nSys: %i Gyro: "
        "%i Accel: %i Mag: %i X_acc: %f Y_acc: %f Z_acc: %f\n ",
         substate.imu.yaw,	substate.imu.roll,	substate.imu.pitch,
         substate.imu.p, 		substate.imu.q,			substate.imu.r,
         substate.imu.sys,	substate.imu.gyro,	substate.imu.accel,
         substate.imu.mag,	substate.imu.x_acc,	substate.imu.y_acc,
         substate.imu.z_acc);
*/
      
      // Write a newline
//      output << std::endl;
      
      // Write to general log file
      genlog << output.str();
      
      // Write to output
      std::cout << output.str();
      
      
      /* Generate an output */
      
      // Yaw controller info
      gettimeofday(&now, NULL);
      double timestamp = (now.tv_sec - start.tv_sec)*1000 + 
                         (now.tv_usec - start.tv_usec)/1000;
      double yaw = substate.imu.yaw;
      double setpoint = yaw_pid.setpoint;
      
      // Write to file
      yawlog << timestamp << "," << yaw << "," << setpoint << ",";
      yawlog << portmotorspeed << "," << starmotorspeed << std::endl;
      
    }
		auv_msleep(1000/LOG_RATE);
	}
	
	// Close log files
	genlog.close();
	yawlog.close();
	
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
  float basespeed = 0.2;
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
	depth_pid.period = -1; 			//not cyclical controller


	// read IMU values from fifo file
	substate.imu = read_imu_fifo();

	// Read pressure values
	ms5837 = read_pressure_fifo();

	// Set setpoint to current heading
  if( substate.imu.yaw > 180 )
    yaw_pid.setpoint = substate.imu.yaw - 360;
  else
    yaw_pid.setpoint = substate.imu.yaw;

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
      portmotorspeed = set_motor(0, basespeed - motorpercent);

      // Set starboard motor
      starmotorspeed = set_motor(1, basespeed + motorpercent);

      //set vertical thruster
      //vertmotorspeed = set_motor(2, depthpercent);
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


/******************************************************************************
 * Safety Thread
 *
 * Shuts down AUV if vehicle goes belows 10m, temperature gets too high, or
 * water intrusion is detected
 *****************************************************************************/
void *safety_thread(void* arg)
{
	printf("Safety Thread Started\n");

	// open a log to record reasons for shutting down
	std::ofstream logFile;
	logFile.open("safety.log");

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
			logFile << "Shut down due to max depth being reached\n";
			logFile << "Stop depth: STOP_DEPTH\n";
			logFile << "Current depth: " << ms5837.depth << "\n";
			printf("\nWe're too deep! Shutting down...\n");
			continue;
		}
		// Check battery compartment temperature
		float _temp = read_temp_fifo();
		if( _temp > STOP_TEMP )
		{
			substate.mode = STOPPED;
			logFile << "Shut down due to max battery temp being reached\n";
			logFile << "Stop temp: STOP_TEMP\n";
			logFile << "Current temp: " << _temp << "\n";
			printf("\nMax battery temp reached: ( %5.2f C)! Shutting down...\n",_temp);
			continue;
		}

		// check pi cpu tem
		// temp is multiplied by 1000 in raspbian OS
		float _cpu_temp = read_cpu_temp();
		if (_cpu_temp > 80000) {
			logFile << "Shut down due to max cpu temp being reached\n";
			logFile << "Stop temp: 80 C\n";
			logFile << "Current temp: " << _cpu_temp << "\n";
			printf("CPU is above 80 C. Shutting down...\n");
			substate.mode = STOPPED;
		}
		// Check for leak
		if( digitalRead(LEAKPIN) == HIGH )
		{
			substate.mode = STOPPED;
			logFile << "Shut down due to leak\n";
			printf("\nLEAK DETECTED! Shutting down...\n");
			continue;
		}

		// Check IMU accelerometer for too much pitch
		if(  (float)fabs(substate.imu.pitch) > PITCH_LIMIT )
		{
		  substate.mode = STOPPED;
			logFile << "Shut down due to excessive pitch (PITCH_LIMIT deg)\n";
			char accel[100];
			sprintf(accel, "Pitch: %5.2f  Roll: %5.2f\n",
				substate.imu.pitch, substate.imu.roll);
		}

		// Check IMU accelerometer for too much roll
		if( (float)fabs(substate.imu.roll) > ROLL_LIMIT )
		{
		  substate.mode = STOPPED;
			logFile << "Shut down due to excessive roll (ROLL_LIMIT deg)\n";
			char accel[100];
			sprintf(accel, "Pitch: %5.2f  Roll: %5.2f\n",
				substate.imu.pitch, substate.imu.roll);
		}

		// Check IMU accelerometer for collision (1+ g detected)
		if(  (float)fabs(substate.imu.x_acc) > 1.0*GRAVITY
			|| (float)fabs(substate.imu.y_acc) > 1.0*GRAVITY
			|| (float)fabs(substate.imu.z_acc) > 2.0*GRAVITY )
		{
			substate.mode = STOPPED;
			logFile << "Shut down due to excessive acceleration (1 g)\n";
			char accel[100];
			sprintf(accel, "X Acc: %5.2f  Y Acc: %5.2f  Z Acc: %5.2f\n",
				substate.imu.x_acc, substate.imu.y_acc, substate.imu.z_acc);
			logFile << accel;
			printf("\nCollision detected. Shutting down...");
			continue;
		}
		
		// Sleep a bit
		auv_msleep(1000/SAFETY_RATE);
	}
	// close log file
	logFile.close();

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
  float _kp = 0.0003, _ki = 0, _kd = 0;

  // Wait a until everything is initialized before starting
  while(substate.mode == INITIALIZING)
  {
    // Waiting...
    auv_msleep(100);
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
		depth_pid.perr = 0;
		depth_pid.ierr = 0;
		depth_pid.derr = 0;

    // Start RUNNING again
    substate.mode = RUNNING;

    // Restart timer!
	  //gettimeofday(&start, NULL);

		// Wait for mode to pause again
		while (substate.mode == RUNNING) {
			auv_msleep(100);
		}

    auv_msleep(1000);
  }

  // Exit thread
  pthread_exit(NULL);
 }
