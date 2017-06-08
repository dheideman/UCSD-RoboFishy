/******************************************************************************
 * initialize_sensors.cpp
 *
 * Contains the scripps_auv_init() function
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * int scripps_auv_init(void)
 *
 * Initializes the IMU, pressure sensor, and temperature sensor
***************************************************************************/

int initialize_sensors(void)
{
	start_read_imu();			// start IMU
	usleep(100000);
	start_read_pressure();		// start pressure sensor
	usleep(100000);
	start_read_temp();			// start temperature sensor
	usleep(100000);
	signal(SIGINT, ctrl_c);		// capture ctrl+c and exit
	return 0;
}
