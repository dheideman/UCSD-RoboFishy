/******************************************************************************
* MS5837.cpp
*
* File to run Initialization and reading files on the MS5837 Pressure Sensor
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * pressure_calib_t init_pressure_sensor
 *
 * Initializes pressure sensor and returns structure of 6 coefficients
***************************************************************************/

pressure_calib_t init_pressure_sensor(void)
{
	// Initialize Python interpreter
	Py_Initialize();

	// Create struct to hold calibration data
	pressure_calib_t pressure_calib;

	// Create pointers to python object
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue;

	// Input name of python source file
	pName = PyString_FromString("MS5837");

	// Stuff
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"/home/pi/UCSD-RoboFishy/Working/Electronics/Mapper\")");
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
 * void start_read_pressure
 *
 * Starts pressure reading python program
***************************************************************************/

void start_read_pressure(void)
{
	FILE* fd = fopen("python read_pressure.py", "r");
	PyRun_SimpleFile(fd,"python read_pressure.py");
	return;
}

/***************************************************************************
 * ms5837_t ms5837_read
 *
 * Read pressure values from MS5837 pressure sensor
***************************************************************************/

ms5837_t ms5837_read(pressure_calib_t pressure_calib)
{
	// Initialize Python interpreter
	Py_Initialize();

	// Create struct to hold pressure data
	ms5837_t ms5837;

	// Create pointers to Python object
	PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;

	// Input name of Python source file
	pName = PyString_FromString("MS5837_example");
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"/home/pi/UCSD-RoboFishy/Working/Electronics/Mapper\")");
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
