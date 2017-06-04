#include "Libraries.h"

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
 * calib_t init_ms5837
 *
 * Description
***************************************************************************/

calib_t init_ms5837(void)
{
	Py_Initialize();
	calib_t calib; // create struct to hold calibration data

	// create pointers to python object
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue;
	pName = PyString_FromString("MS5837"); // input name of python source file
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"/home/pi/UCSD-RoboFishy/Working/Electronics/MainScript\")");
	pModule = PyImport_Import(pName);
	pDict = PyModule_GetDict(pModule);
	pFunc = PyDict_GetItemString(pDict, "init_press");
	if (PyCallable_Check(pFunc))
	{
		pValue = PyObject_CallObject(pFunc, NULL);
		PyArg_ParseTuple(pValue,"ffffff", &calib.C1, &calib.C2, &calib.C3,&calib.C4, &calib.C5, &calib.C6);
		Py_DECREF(pValue);
	}
	else
	{
		PyErr_Print();
	}
	Py_DECREF(pModule);
	Py_DECREF(pName);
	Py_Finalize();
	return calib;
};

/***************************************************************************
 * ms5837_t ms5837_read
 *
 * Description
***************************************************************************/

ms5837_t ms5837_read(calib_t arg_in)	//	read pressure values from MS5837 pressure sensor
{
	ms5837_t ms5837;

	Py_Initialize();

	calib_t calib = arg_in; // create struct to hold calibration data
	PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;

	pName = PyString_FromString("MS5837_example"); // input name of python source file
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"/home/pi/UCSD-RoboFishy/Working/Electronics/MainScript\")");
	pModule = PyImport_Import(pName);
	pDict = PyModule_GetDict(pModule);
	pFunc = PyDict_GetItemString(pDict, "read_press");

	if (PyCallable_Check(pFunc))
	{

		pArgs = Py_BuildValue("ffffff", calib.C1,calib.C2,calib.C3,calib.C4,calib.C5,calib.C6);

		pValue = PyObject_CallObject(pFunc, pArgs);

		//PyArg_ParseTuple(pValue,"ff", &ms5837.temperature, &ms5837.pressure);
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
	int channels[]  = {CHANNEL_1, CHANNEL_2, CHANNEL_3};
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
int yaw_controller()
{
// read IMU values into sstate
		bno055 = bno055_read();
		
}
/***************************************************************************
 * int set_motors()
 *
 * Cleans up the AUV script, shuts down the motors and closes all threads
***************************************************************************/

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
					/*if(mix_controls(u[0],u[1],u[2],u[3],&new_esc[0],2)<0)
					{
						printf("ERROR, mixing failed\n");
					}
					for(i = 0; i < 2; i++)
					{
						if(new_esc[i]>1.0)
						{
							new_esc[i]=1.0;
						}
						else if(new_esc[i]<-1.0)
						{
							new_esc[i]=-1.0;
						}
					}*/


					// ESC outputs //
					sstate.esc_out = u[2];
					//sstate.esc_out[0] = new_esc[0];		// port motor (CCW)
					//sstate.esc_out[1] = new_esc[1];		// starboard motor (CW)

					// print ESC values
					//printf("ESC1: %f ESC2: %f \n ", sstate.esc_out[0],sstate.esc_out[1]);

					// saturate motor output values //
					if(sstate.yaw[0]>180) // port motor (CCW)
					{
						output_port = -26.18*100*sstate.esc_out+2630;
						if(output_port<(2630-(0.2*(2630-12))))	// set motor output at 20% of max for testing purposes (20% = 2106.4)
						{
							output_port = 2630-(0.2*(2630-12));		// for testing purposes
							printf("Port PWM Output1: %f\n", output_port);
						}

						output_starboard = 3155.4-(2630-output_port)/(2630-12)*(4905-2718); // starboard motor output = base 20% minus percentage that port motor increased by
						if(output_starboard<(2718+0.1*(4905-2718)))
						{
							output_starboard =	2718+0.1*(4905-2718); // set starboard motor output to no less than 10%
						}

						output_port = output_port-0.2*(2630-12);			// port motor max at 40%
						pwmWrite(PIN_BASE+motor_channels[0], output_port);	// port motor at base 20% + yaw control output
						pwmWrite(PIN_BASE+motor_channels[1], output_starboard);				// starboard motor at base 20%
					}
					else	// starboard motor (CW)
					{
						output_starboard = 13.77*100*-sstate.esc_out+2718;
						if(output_starboard>(2718+(0.2*(4905-2718)))) // set motor output at 20% of max for testing purposes (20% = 3155.4)
						{
							output_starboard = 2718+(0.2*(4905-2718));	// for testing purposes
							//output_starboard = 4095;
							printf("Starboard PWM Output1: %f\n", output_starboard);
						}

						output_port = 2106.4 - (output_starboard-2718)/(4905-2718)*(2630-12); // port motor output = base 20% minus percentage that starboard motor increased by
						if(output_port>(2630-(0.1*(2630-12))))
						{
							output_port = 2630-(0.1*(2630-12));		// set port motor output to no less than 10%
						}

						output_starboard = output_starboard+0.2*(4905-2718);	// starboard motor max at 40%
						pwmWrite(PIN_BASE+motor_channels[1], output_starboard); //	starboard motor output = base 20% + yaw control output
						pwmWrite(PIN_BASE+motor_channels[0], output_port);				// port motor at base 20%
					}

					// print motor PWM outputs //
					printf("Port PWM Output2: %f Starboard PWM Output2: %f\n",
						output_port, output_starboard);