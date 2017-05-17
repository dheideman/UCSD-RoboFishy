#include "pca9685.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <Python.h>
#include <time.h>


//#define PIN_BASE 300
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 370
#define PCA9685_ADDR 0x40


// struct for hold ms5837 calibration values
typedef struct{
	float C1, C2, C3, C4, C5, C6;
} calib_t;

// struct for holding ms5837 return values
typedef struct {
	float pressure, temperature;
} ms5837_t;

// struct for holding bno055 return values
typedef struct {
	float yaw, roll, pitch, p, q, r;
	int sys, gyro, accel, mag;
}bno055_t;



// Functions for setting motor PWM with PCA9685
int calcTicks(float impulseMs, int hertz);
int initialize_motors(int channels[4], float freq);
int saturate_number(float* val, float min, float max);


// Functions for Reading MS5837 Pressure sensor
calib_t init_ms5837(); // initialize ms5837
ms5837_t ms5837_read(calib_t arg_in); // read values from ms5837

// Functions for Reading BNO055 IMU
void start_Py_bno055(void); // start Python background process
bno055_t bno055_read(void); // read values from bno055
