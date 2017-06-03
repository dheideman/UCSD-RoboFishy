	#include "Libraries.h"

///////////////////////////////////////////////////////////////////////////////////
////////////////////////////// PCA9685 FUNCTIONS //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

// calculate PWM input //
int calcTicks(float impulseMs, int hertz)
{
	float cycleMs = 1000.0f / hertz;									// calculate period
	//return (int)(MAX_PWM * (0.45*impulseMs+1.64) / cycleMs + 0.5f);		// formula for setting PWM output
	return (int)(MAX_PWM * (0.45*impulseMs+1.64) / cycleMs + 0.5f);		// formula for setting PWM output
}

// initialize motors function //
int initialize_motors(int channels[4], float freq)
{
	int i;
	int fd = pca9685Setup(PIN_BASE, PCA9685_ADDR, HERTZ);				// setup PCA9685 board
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
				//printf("%i\n",calcTicks(0,HERTZ));
			}
		}

	/*
	for( i = 0; i < 3; i++ )
	{
		pwmWrite (PIN_BASE+channels[i], 2631);
		//pwmWrite (PIN_BASE+channels[i], 1500);
		//usleep(10000);
		//sleep(3);
	}*/
	//usleep(100000);
	return fd;
}

// saturate values //
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


///////////////////////////////////////////////////////////////////////////////////
////////////////////////////// MS5837 FUNCTIONS ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

calib_t init_ms5837(void)
{
	Py_Initialize();
	calib_t calib; // create struct to hold calibration data
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

void start_Py_ms5837(void)
{
	char cmd[50];
    strcpy(cmd,"python MS5837_example.py & exit");
    system(cmd);
    return;
}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// BNO055 FUNCTIONS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void start_Py_bno055(void)	//	start bno055_read.py code
{
    char cmd[50];
    strcpy(cmd,"python bno055_read.py & exit");
    system(cmd);
    return;
}

bno055_t bno055_read(void)	// read values from bno055 IMU
{ 
	bno055_t bno055;
	char buf[1000];
	FILE *fd = fopen( "/home/pi/UCSD-RoboFishy/Working/Electronics/bno055_fifo.txt", "r");
	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f %f %f %f %f %f %i %i %i %i", 
	&bno055.yaw,&bno055.roll,&bno055.pitch,
	&bno055.q, &bno055.p, &bno055.r,
	&bno055.sys,&bno055.gyro,&bno055.accel,&bno055.mag);
	return bno055;
}

///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// DS18B20 Functions ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void start_Py_ds18b20(void)		//	start temperature_sensor_code.py code
{
	char cmd[50];
	strcpy(cmd,"python temperature_sensor_code.py & exit");
	system(cmd);
	return;
}

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

// Initialization Script: This script runs all of the initialization functions // 
int scripps_auv_init(void)
{
	start_Py_bno055();
	sleep(10);
	//start_Py_ms5837();
	start_Py_ds18b20();
	signal(SIGINT, ctrl_c);	
	return 0;
}

// state variable for loop and thread control //
enum state_t state = UNINITIALIZED;

// get system state //
enum state_t get_state()
{
	return state;
}

// set system state //
int set_state(enum state_t new_state)
{
	state = new_state;
	return 0;
}

// catch the contrl-c and close the script //
void ctrl_c(int signo)
{
	if (signo == SIGINT)
	{
		set_state(EXITING);
		printf("\nreceived SIGINT Ctrl-C\n");
 	}
}

// Clean up AUV script: This function stops the motors and closes all the threads //
int cleanup_auv()
{
	set_state(EXITING);
	usleep(500000); // let final threads clean up
	int channels[]  = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4};
	int i;
	for( i = 0; (i < 4); i = i+1 )
	{
		//pwmWrite (PIN_BASE+channels[i], calcTicks(0, HERTZ));
		pwmWrite (PIN_BASE+channels[i], 2674);		// set motor outputs to 0
		usleep(10000);
	}
	printf("\nExiting Cleanly\n");
	return 0;
}