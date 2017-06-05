#include "Mapper.h"

// state variable for loop and thread control //
enum state_t state = UNINITIALIZED;

/***************************************************************************
 * int initialize_motors
 *
 * Description
***************************************************************************/

// initialize motors function //
int initialize_motors(int channels[4], float freq)
{
	int i;
	int fd = pca9685Setup(PIN_BASE, PCA9685_ADDR, HERTZ); // setup PCA9685 board
	if (fd < 0)
	{
		printf("Error in setup\n");
		return fd;
	}
	pca9685PWMReset(fd);												// reset all output
	//usleep(10);

	// imported from motor.c test code //
	int active=1;
		while (active)
		{
			for( i = 0; i < 3; i++ )
			{
				//pwmWrite (PIN_BASE+i, calcTicks(0,HERTZ));
				pwmWrite (PIN_BASE+i, 2674);	//send input signal that is low enough to reach the
												//"neutral" or power-off area in order to arm the ESC (long beep); green LED on ESC will light up
				delay(1000);
				active=0;
			}
		}
	//usleep(100000);
	return fd;
}

/***************************************************************************
 * int saturate_number
 *
 * Description
***************************************************************************/

int saturate_number(float* val, float min, float max)
{
	if(*val>max)
	{
		*val = max;
		return 1;
	}
	else if(*val<min)
	{
		*val = min;
		return 1;
	}
	return 0;
}


/***************************************************************************
 * pressure_calib_t init_ms5837
 *
 * initializes pressure sensor
 *
 * Returns structure of 6 coefficients
***************************************************************************/

pressure_calib_t init_ms5837(void)
{
	Py_Initialize();
	pressure_calib_t pressure_calib; // create struct to hold calibration data

	// create pointers to python object
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue;

	// input name of python source file
	pName = PyString_FromString("MS5837");

	// stuff
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"/home/pi/UCSD-RoboFishy/Working/Electronics/MainScript\")");
	pModule = PyImport_Import(pName);
	pDict = PyModule_GetDict(pModule);
	pFunc = PyDict_GetItemString(pDict, "init_press");
	if (PyCallable_Check(pFunc))
	{
		pValue = PyObject_CallObject(pFunc, NULL);
		PyArg_ParseTuple(pValue,"ffffff",
			&pressure_calib.C1, &pressure_calib.C2, &pressure_calib.C3,
			&pressure_calib.C4, &pressure_calib.C5, &pressure_calib.C6);
		Py_DECREF(pValue);
	}
	else
	{
		PyErr_Print();
	}
	Py_DECREF(pModule);
	Py_DECREF(pName);
	Py_Finalize();
	return pressure_calib;
};

/***************************************************************************
 * ms5837_t ms5837_read
 *
 * Read pressure values from MS5837 pressure sensor
***************************************************************************/

ms5837_t ms5837_read(pressure_calib_t pressure_calib)
{
	ms5837_t ms5837;

	Py_Initialize();

	PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;

	pName = PyString_FromString("MS5837_example"); // input name of python source file
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"/home/pi/UCSD-RoboFishy/Working/Electronics/MainScript\")");
	pModule = PyImport_Import(pName);
	pDict = PyModule_GetDict(pModule);
	pFunc = PyDict_GetItemString(pDict, "read_press");

	if (PyCallable_Check(pFunc))
	{
		pArgs = Py_BuildValue("ffffff", pressure_calib.C1,pressure_calib.C2,pressure_calib.C3,
			pressure_calib.C4,pressure_calib.C5,pressure_calib.C6);

		pValue = PyObject_CallObject(pFunc, pArgs);

		PyArg_ParseTuple(pValue,"ff", &ms5837.pressure);
		Py_DECREF(pArgs);
		Py_DECREF(pValue);
	}
	else
	{
		PyErr_Print();
	}

	Py_DECREF(pModule);
	Py_DECREF(pName);
	Py_Finalize();
	return ms5837;
};

/***************************************************************************
 * void start_Py_ms5837
 *
 * Description
***************************************************************************/

void start_Py_ms5837(void)
{
	FILE* fd = fopen("python MS5837_example.py", "r");
	PyRun_SimpleFile(fd,"python MS5837_example.py");
	return;
}


////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// BNO055 FUNCTIONS //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/***************************************************************************
 * void start_Py_bno055
 *
 * Description
***************************************************************************/

void start_Py_bno055(void)	//	start bno055_read.py code
{
	// clear fifo file
	FILE* fd = fopen("bno055_read.py", "r");
	PyRun_SimpleFile(fd,"bno055_read.py");
	//nanosleep(100*1000000);
	FILE* fifo = fopen("bno055_fifo.txt","r");
	fclose(fifo);

	// check if fifo file has numbers in it
	return;
}

/***************************************************************************
 * bno055_t bno055_read
 *
 * Description
***************************************************************************/

bno055_t bno055_read(void)	// read values from bno055 IMU
{
	bno055_t bno055;
	char buf[1000];
	FILE *fd = fopen( "bno055_fifo.txt", "r");
	//FILE *fd = fopen( "/home/pi/UCSD-RoboFishy/Working/Electronics/bno055_fifo.txt", "r");

	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f %f %f %f %f %f %i %i %i %i",
				 &bno055.yaw,&bno055.roll,&bno055.pitch,
				 &bno055.q, &bno055.p, &bno055.r,
				 &bno055.sys,&bno055.gyro,&bno055.accel,
				 &bno055.mag);

	return bno055;
}

///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// DS18B20 Functions ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/***************************************************************************
 * void start_Py_ds18b20
 *
 * Writes temperature values from the DS18B20 temperature sensor into
 * ds18b20_fifo.fifo
***************************************************************************/

void start_Py_ds18b20(void)		//	start temperature_sensor_code.py code
{
	FILE* fd = fopen("python temperature_sensor_code.py", "r");
	PyRun_SimpleFile(fd,"python temperature_sensor_code.py");
	return;
}

/***************************************************************************
 * ds18b20_t ds18b20_read(void)
 *
 * Reads temperature values from ds18b20_fifo.fifo
***************************************************************************/

ds18b20_t ds18b20_read(void)	// read values from ds18b20 temperature sensor
{
	ds18b20_t ds18b20;
	char buf[1000];
	FILE *fd = fopen( "/home/pi/UCSD-RoboFishy/Working/Electronics/ds18b20_fifo.fifo", "r");
	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f",&ds18b20.temperature);
	return ds18b20;
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////Setup and Shutdown//////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/***************************************************************************
 * int scripps_auv_init(void)
 *
 * Initializes the IMU, pressure sensor, and temperature sensor
***************************************************************************/

int scripps_auv_init(void)
{
	start_Py_bno055();			// start IMU
	sleep(10);
	//start_Py_ms5837();		// start pressure sensor
	start_Py_ds18b20();			// start temperature sensor
	signal(SIGINT, ctrl_c);		// capture ctrl+c and exit
	return 0;
}


/***************************************************************************
 * state_t get_state()
 *
 * Gets the AUV's current state
***************************************************************************/

state_t get_state()
{
	return state;
}

/***************************************************************************
 * int set_state(enum state_t new_state)
 *
 * Sets the AUV state
***************************************************************************/

int set_state(enum state_t new_state)
{
	state = new_state;
	return 0;
}

/***************************************************************************
 * void ctrl_c(int signo)
 *
 * Captures a user-entered ctrl+c and exits the script
***************************************************************************/
void ctrl_c(int signo)
{
	if (signo == SIGINT)
	{
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
	}
}

/***************************************************************************
 * int cleanup_auv()
 *
 * Cleans up the AUV script, shuts down the motors and closes all threads
***************************************************************************/
int cleanup_auv()
{
	set_state(EXITING);
	usleep(500000); // let final threads clean up
	int channels[]	= {CHANNEL_1, CHANNEL_2, CHANNEL_3};
	int i;
	for( i = 0; (i < 3); i = i+1 )
	{
		//pwmWrite (PIN_BASE+channels[i], calcTicks(0, HERTZ));
		pwmWrite (PIN_BASE+channels[i], 2674);		// set motor outputs to 0
		usleep(10000);
	}
	printf("\nExiting Cleanly\n");
	return 0;
}

/******************************************************************************
* int yaw_controller()
*
* Takes in readings from IMU and calculates a percentage (-1 to 1)
******************************************************************************/
float yaw_controller()
{
	// control output //
	if(bno055.yaw<180) // AUV is pointed right
	{
		// u[2] is negative
		motor_percent = yaw_pid.kp*(bno055.yaw - yaw_pid.setpoint); //+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
	}
	else		// AUV is pointed left
	{
		// u[2] is positive
		motor_percent = yaw_pid.kp*(yaw_pid.setpoint-(bno055.yaw-360)); //+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
	}
	// saturate yaw controller //
	if(u[2]>YAW_SAT)
	{
		motor_percent=YAW_SAT;
	}
	else if(motor_percent<-YAW_SAT)
	{
		motor_percent=-YAW_SAT;
	}

	//set current yaw to be the old yaw
	yaw_pid.oldyaw=bno055.yaw;

	return motor_percent;
}
/***************************************************************************
 * int set_motors()
 *
 * Takes in a value from -1 to 1 (-100 to +100%) and sets the motor
 * outputs accordingly
***************************************************************************/
int set_motors(int motor_num, float speed)
{
	int motor_num;				// indicates which motor to write to
								// port = 0, starboard = 1, vert = 2
	float motor_output;			// feeds the necessary PWM to the motor
	float per_run = 0.2;		// percentage of full PWM to run at
	//float min_per_run = 0.1;	// minimum percentage of full PWM to run at
	int port_range = 2618;		// port motor range
	int starboard_range = 2187; // starboard motor range

	// speed = (-) ----> AUV pointed right (starboard) (range: 2718-4095)
	// speed = (+) ----> AUV pointed left (port) (range: 12-2630)

	// Calculate motor output //
	if( speed < 0 )
	{
		motor_output = 2630 - per_run*port_range;				// set motor output

		// saturate motor output at 20% //de
		if( motor_output < (2630-per_run*port_range) )
		{
			motor_output = (2630-per_run*port_range);
		}
		pwmWrite(motor_num, motor_output);
	}
	if( speed > 0 )
	{
		motor_output = 2718 + per_run*starboard_range;			// set motor output

		// saturate motor output at 20% //
		if( motor_output > 2718 + per_run*starboard_range )
		{
			motor_output = 2718 + per_run*starboard_range;
		}
	}
	else
		motor_output = 2674;	// turn off motor
}
