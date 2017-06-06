/******************************************************************************
* Scripps_AUV_Init.cpp
*
* Contains the scripps_auv_init() function
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * int scripps_auv_init(void)
 *
 * Initializes the IMU, pressure sensor, and temperature sensor
***************************************************************************/

int scripps_auv_init(void)
{
	start_Py_bno055();			// start IMU
	sleep(1);					// sleep for 1 second
//	start_Py_ms5837();			// start pressure sensor
//	start_Py_ds18b20();			// start temperature sensor
	signal(SIGINT, ctrl_c);		// capture ctrl+c and exit
	return 0;
}
