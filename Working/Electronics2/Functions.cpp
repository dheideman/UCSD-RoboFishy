#include "Libraries.h"

// PCA9685 FUNCTIONS //

// calculate PWM input
int calcTicks(float impulseMs, int hertz)
{
	float cycleMs = 1000.0f / hertz;									// calculate period
	return (int)(MAX_PWM * (0.45*impulseMs+1.64) / cycleMs + 0.5f);		// formula for setting PWM output
}

// initialize motors function
int initialize_motors(int channels[4], float freq){
	int i;
	int fd = pca9685Setup(PIN_BASE, PCA9685_ADDR, HERTZ);				// setup PCA9685 board
	pca9685PWMReset(fd);												// reset all output
	//usleep(10);
	for( i = 0; (i < 4); i = i+1 ){
		pwmWrite (PIN_BASE+channels[i], 1500);
		//usleep(10000);
		//sleep(3);
	}
	//usleep(100000);
	return fd;
}

// saturate values
int saturate_number(float* val, float min, float max){
	if(*val>max){
		*val = max;
		return 1;
	}
	else if(*val<min){	
		*val = min;
		return 1;
	}
	return 0;
}


// MS5837 FUNCTIONS //

calib_t init_ms5837(void){
	printf("%s\n", "start of calib_t" );
	Py_Initialize();
	calib_t calib; // create struct to hold calibration data
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue;
	
	pName = PyString_FromString("MS5837"); // input name of python source file
	/*if (pName==NULL){ 
		printf("%s\n", "NULL");
	}
	else{
		printf("%s\n", "not NULL");
	}*/
	printf("%s\n", "After pName" );
	PyRun_SimpleString("import sys");
	printf("%s\n","Right before syspathappend" );
	PyRun_SimpleString("sys.path.append(\"/home/pi/UCSD-RoboFishy/Working/Electronics/MainScript\")");
	printf("%s\n", "After syspathappend" );
	pModule = PyImport_Import(pName);
	if (pModule==NULL){ 
		printf("%s\n", "NULL");
	}
	else{
		printf("%s\n", "not NULL");
	}
	printf("%s\n", "After pModule pName" );
	pDict = PyModule_GetDict(pModule);
	printf("%s\n","After GetDict" );
	pFunc = PyDict_GetItemString(pDict, "init_press");
	printf("%s\n", "after GetItemString");
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

ms5837_t ms5837_read(calib_t arg_in){
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
	
	if (PyCallable_Check(pFunc)) {

		pArgs = Py_BuildValue("ffffff", calib.C1,calib.C2,calib.C3,calib.C4,calib.C5,calib.C6);
		
		pValue = PyObject_CallObject(pFunc, pArgs);
		
		PyArg_ParseTuple(pValue,"ff", &ms5837.temperature, &ms5837.pressure);
		Py_DECREF(pArgs);
		Py_DECREF(pValue);
	}
	else {
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


// BNO055 FUNCTIONS //

void start_Py_bno055(void){
    char cmd[50];
    strcpy(cmd,"python bno055_read.py & exit");
    system(cmd);
    return;
}

bno055_t bno055_read(void){ // read values from bno055
	bno055_t bno055;
	char buf[1000];
	FILE *fd = fopen( "/home/pi/UCSD-RoboFishy/Working/Electronics/bno055_fifo.fifo", "r");
	fgets(buf,1000,fd);
	fclose(fd);
	sscanf(buf,"%f %f %f %f %f %f %i %i %i %i", 
	&bno055.yaw,&bno055.roll,&bno055.pitch,
	&bno055.q, &bno055.p, &bno055.r,
	&bno055.sys,&bno055.gyro,&bno055.accel,&bno055.mag);
	return bno055;
}
