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
<<<<<<< HEAD
	start_Py_bno055();				// start IMU Python script
	sleep(10);
	//start_Py_ms5837();			// start pressure sensor
	start_Py_ds18b20();				// start temperature sensor
=======
	start_Py_bno055();			// start IMU
	sleep(1);
	//start_Py_ms5837();		// start pressure sensor
	start_Py_ds18b20();			// start temperature sensor
>>>>>>> c35849547d66cc742094ed2eec39df1b9c941c14
	signal(SIGINT, ctrl_c);		// capture ctrl+c and exit
	return 0;
}
