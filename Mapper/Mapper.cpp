/******************************************************************************
*	Main script for the 2017 RoboFishy Scripps AUV
******************************************************************************/

#include "Mapper.h"

/******************************************************************************
* Controller Gains
******************************************************************************/


/******************************************************************************
 * Global Variables
******************************************************************************/

// Holds the latest pressure value from the MS5837 pressure sensor
ms5837_t ms5837;

// Holds the constants and latest errors of the yaw pid controller
pid_data_t yaw_pid;

// Holds the constants and latest errors of the depth pid controller
pid_data_t depth_pid;

// Motor channels
int motor_channels[] = {CHANNEL_1, CHANNEL_2, CHANNEL_3};

// Holds the latest temperature value from the temperature temperature sensor
float batt_temp;

// setmotor intialization
float portmotorspeed = 0;
float starmotorspeed = 0;

// Start time for stop timer
struct timeval mainstart, setptstart, now;


/******************************************************************************
* Main Function
******************************************************************************/

int main(int argc, char** argv)
{
  // set mode to INITIALIZING
  substate.mode = INITIALIZING;
  
	// capture ctrl+c and exit
	signal(SIGINT, ctrl_c);

	// Set up RasPi GPIO pins through wiringPi
	wiringPiSetup();
	
	// Start camera thread early
	initializeTAttr();
	pthread_t cameraThread;
	pthread_create (&cameraThread, &tattrhigh, takePictures, NULL);

  // Set laser pin to output
//  pinMode(3, OUTPUT);

	// Check if AUV is initialized correctly
	if( initialize_sensors() < 0 )
	{
		return -1;
	}
	printf("\nAll components are initialized\n");
	substate.mode = INITIALIZING;
	substate.laserarmed = ARMED;

	// Initialize Thread locks
	initializeSubImagesLock(&subimages);
	initializeOdomDataLock(&odomdata);

	printf("Starting Threads\n");

	// Thread handles
	pthread_t navigationThread;
	pthread_t logThread;
	pthread_t safetyThread;
	//pthread_t disarmlaserThread;
	pthread_t uiThread;
	pthread_t rangeThread;
	pthread_t odometryThread;


	// Create threads using modified attributes
//	pthread_create (&disarmlaserThread, &tattrlow, disarmLaser, NULL);
	pthread_create (&safetyThread, &tattrlow, safety_thread, NULL);
	pthread_create (&logThread, &tattrlow, log_thread, NULL);
	pthread_create (&navigationThread, &tattrmed, navigation_thread, NULL);
	pthread_create (&rangeThread, &tattrmed, rangeFinder, NULL);
	pthread_create (&odometryThread, &tattrmed, visualOdometry, NULL);
//	pthread_create (&uiThread, &tattrmed, userInterface, NULL);

  // Destroy the thread attributes
 	destroyTAttr();

  printf("Threads started\n");

  auv_msleep(100);

	// iterate counter
	int iterator = 0;

	// Prompt for run time
	int run_time = 20;
  if( argc >= 2 )
  {
    run_time = atoi( argv[1] );
  }

//	std::cout << "Number of Seconds to run: ";
//	std::cin >> run_time;

	// Start timers!
	gettimeofday(&mainstart,  NULL);
  gettimeofday(&setptstart, NULL);

	yaw_pid.setpoint = substate.imu.yaw;
  if(yaw_pid.setpoint > 180) yaw_pid.setpoint -= 360;

	// We're ready to run.
  substate.mode = RUNNING;

  // Run main while loop, wait until it's time to stop
	while(substate.mode != STOPPED)
	{
		// Check if we've passed the stop time
		gettimeofday(&now, NULL);
 		if((now.tv_sec - mainstart.tv_sec) > run_time)
 			substate.mode = STOPPED;

    if(substate.mode == RUNNING)
    {
      // Change the setpoint every LEG_TIME seconds
      if((now.tv_sec + now.tv_usec/1000000 - (setptstart.tv_sec + setptstart.tv_usec/1000000)) > LEG_TIME)
      {
        // Set new setpoint
        yaw_pid.setpoint += DELTA_SETPOINT;
        if(yaw_pid.setpoint > 180) yaw_pid.setpoint -= 360;

	      // reset timer
	      gettimeofday(&setptstart, NULL);

        // Increment iterator
        iterator++;
      } // end if difftime

    } // end if RUNNING

		// Sleep a little
		auv_msleep(20);
	}

	// Exit cleanly
	cleanup_auv();

  // Write last two images to file
  imwrite("lastbrightimage.jpg",subimages.brightframe);
  imwrite("lastdarkimage.jpg",subimages.darkframe);

  return 0;
}
