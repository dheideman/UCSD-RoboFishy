#include "pca9685.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <Python.h>
#include <time.h>
#include <signal.h>		// capture control-c

#define PCA9685_ADDR 0x40
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 400
#define PCA9685_ADDR 0x40
#define CHANNEL_1 0		// roll
#define CHANNEL_2 1		// pitch
#define CHANNEL_3 2		// yaw
#define CHANNEL_4 3		// speed
#define CHANNEL_5 4		// depth


///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Create Structs ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

// struct for hold ms5837 calibration values
typedef struct
{
	float C1, C2, C3, C4, C5, C6;
}calib_t;

// struct for holding ms5837 return values
typedef struct
{
	//float pressure, temperature;
	float pressure;
}ms5837_t;

// struct for holding bno055 return values
typedef struct
{
	float yaw, roll, pitch, p, q, r;
	int sys, gyro, accel, mag;
}bno055_t;

// struct for holding ds18b20 temperature sensor return values
typedef struct
{
	float temperature;
}ds18b20_t;

//struct for pid controllers (depth and yaw)
typedef struct 
{
	float setpoint;
	float kp, ki, kd;
	float perr, ierr, derr;
};

// Program Flow and State Control
typedef enum state_t
{
	UNINITIALIZED,
	RUNNING,
	PAUSED,
	EXITING
}state_t;


///////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Function Prototypes ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

enum state_t get_state();
int set_state(enum state_t);

// Functions for setting motor PWM with PCA9685
int calcTicks(float impulseMs, int hertz);
int initialize_motors(int channels[4], float freq);
int saturate_number(float* val, float min, float max);


// Functions for Reading MS5837 Pressure Sensor
calib_t init_ms5837(); // initialize ms5837
ms5837_t ms5837_read(calib_t arg_in); // read values from ms5837

// Functions for Reading BNO055 IMU
void start_Py_bno055(void); // start Python background process
bno055_t bno055_read(void); // read values from bno055

// Functions for Reading DS18B20 Temperature Sensor
void start_Py_ds18b20(void); // start Python background process
ds18b20_t ds18b20_read(void);	// read values from ds18b20

// Startup Functions
int scripps_auv_init(void);

//// Cleanup and Shutdown
void ctrl_c(int signo); // signal catcher
int cleanup_auv();		// call at the very end of main()
