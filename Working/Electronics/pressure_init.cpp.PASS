// Filter Values
#define A1 0.3
#define A2 0.4

// Functions for Reading MS5837 Pressure sensor
// initialize ms5837
pressure_calib_t init_ms5837(); 

// read values from ms5837
ms5837_t ms5837_read(pressure_calib_t arg_in);

// MS5837 FUNCTIONS //

pressure_calib_t init_ms5837(void){
	Py_Initialize();
	pressure_calib_t pressure_calib; // create struct to hold calibration data
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue;
	
	pName = PyString_FromString("MS5837"); // input name of python source file
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"/home/pi/Documents/156b/feedback\")");
	pModule = PyImport_Import(pName);
	pDict = PyModule_GetDict(pModule);
	pFunc = PyDict_GetItemString(pDict, "init_press");
	
	if (PyCallable_Check(pFunc)) 
	{
		pValue = PyObject_CallObject(pFunc, NULL);
		PyArg_ParseTuple(pValue,"ffffff", &pressure_calib.C1, &pressure_calib.C2, &calib.C3,&calib.C4, &calib.C5, &calib.C6);
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
	
	pName = PyString_FromString("MS5837"); // input name of python source file
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"/home/pi/Documents/156b/feedback\")");
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


// Global Variables //
calib_t calib; 						// holds the calibration values for the MS5837 pressure sensor
ms5837_t ms5837; 					// holds the latest pressure value from the MS5837 pressure sensor


// get pressure value
calib = init_ms5837();
ms5837 = ms5837_read(calib);
sstate.depth[0] = (ms5837.pressure-1013)*10.197-88.8;
//printf("%f\n", sstate.depth);
// check if depth is above start threshold
sstate.fdepth[0] = A1*(sstate.depth[0]+sstate.depth[1])+A2*sstate.fdepth[1];
if(sstate.fdepth[0]>DEPTH_START)
{
	printf("AUV is waiting to start...\n");
}
else 
{
	printf("AUV started!\n");
}

// set current depth values as old values //
sstate.depth[1] = sstate.depth[0];
sstate.fdepth[1] = sstate.fdepth[0];

//sleep for 50 ms //
usleep(50000);
