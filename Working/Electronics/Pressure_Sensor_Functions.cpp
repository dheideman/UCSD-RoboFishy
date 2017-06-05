/******************************************************************************
* MS5837.cpp
*
* File to run Initialization and reading files on the MS5837 Pressure Sensor
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * pressure_calib_t init_ms5837
 *
 * initializes pressure sensor
 *
 * Returns structure of 6 calibration coefficients
***************************************************************************/
pressure_calib_t init_ms5837(void)
{
	// initialize Python interpreter //
	Py_Initialize();

	// create struct to hold calibration data //
	pressure_calib_t pressure_calib; 

	// create pointers to python object //
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue;

	// input name of python source file //
	pName = PyString_FromString("MS5837");

	// stuff //
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

	// return pressure calibration values //
	return pressure_calib;
};

/***************************************************************************
 * ms5837_t ms5837_read
 *
 * Read pressure values from MS5837 pressure sensor
***************************************************************************/
ms5837_t ms5837_read(pressure_calib_t pressure_calib)
{
	// create struct to hold pressure data //
	ms5837_t ms5837;

	// initialize Python interpreter //
	Py_Initialize();

	// create pointers to python object //
	PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;

	// input name of python source file //
	pName = PyString_FromString("MS5837_example"); 
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

	// return pressure value //
	return ms5837;
};

/***************************************************************************
 * void start_Py_ms5837
 *
 * Starts the pressure-reading Python script
***************************************************************************/
void start_Py_ms5837(void)
{
	// open MS5837_example.py //
	FILE* fd = fopen("python MS5837_example.py", "r");

	// read pressure values //
	PyRun_SimpleFile(fd,"python MS5837_example.py");
	return;
}