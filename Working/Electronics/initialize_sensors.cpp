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
	start_Py_bno055();			// start IMU
	usleep(100000);
	start_Py_ms5837();			// start pressure sensor
	usleep(100000);
	start_Py_ds18b20();			// start temperature sensor
	usleep(100000);
	signal(SIGINT, ctrl_c);	// capture ctrl+c and exit
	return 0;
}
