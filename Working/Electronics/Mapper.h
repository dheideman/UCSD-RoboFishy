#ifndef MAPPER_H
#define MAPPER_H

#define  DEBUG

#include <pca9685.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cstdio>
#include <stdlib.h>
#include <Python.h>
#include <time.h>
#include <signal.h>		// capture control-c


#define PCA9685_ADDR 0x40
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 400
#define PCA9685_ADDR 0x40
#define CHANNEL_1 0		// port motor
#define CHANNEL_2 1		// starboard motor
#define CHANNEL_3 2		// vertical motor

#define MOTOR_0 2674    // motor output is 0

// Core Module
#include "../../Modules/Core/Core.h"


/***************************************************************************
* 
* Create Structs
*
***************************************************************************/

// Struct for hold ms5837 calibration values
typedef struct
{
	float C1, C2, C3, C4, C5, C6;
}pressure_calib_t;

// Struct for holding MS5837 return values
typedef struct
{
	float pressure;
}ms5837_t;

// Struct for holding DS18B20 temperature sensor return values
typedef struct
{
	float temperature;
}ds18b20_t;

// Struct for PID Controllers
typedef struct pid_data_t
{
	float kp, ki, kd;
	float err, i_err;
	float setpoint;
	float old;
	float current;
}pid_data_t;

// Struct for setpoints
typedef struct setpoint_t
{
	float yaw;				// yaw angle in (rad)
	float yaw_rate;			// yaw rate (rad/s)
	float depth;			// z component in fixed coordinate system
	float speed;			// speed setpoint
}setpoint_t;

// Struct for holding current system state
typedef struct system_state_t
{
	float roll;					// current roll angle (rad)
	float pitch[2];				// current pitch angle (rad) 0: current value, 1: last value
	float yaw[2];				// current yaw angle (rad) 0: current value, 1: last value
	float depth[2];				// depth estimate (m)
	float fdepth[2];			// filtered depth estimate (m)
	float speed;				// speed (m/s)

	float p[2];					// first derivative of roll (rad/s)
	float q[2];					// first derivative of pitch (rad/s)
	float r[2];					// first derivative of yaw (rad/s)
	float ddepth;				// first derivative of depth (m/s)

	int sys;		// system calibrations status (0=uncalibrated, 3=fully calibrated)
	int gyro;		// gyro calibrations status (0=uncalibrated, 3=fully calibrated)
	int accel;		// accelerometer calibrations status (0=uncalibrated, 3=fully calibrated)
	int mag;		// magnetometer calibrations status (0=uncalibrated, 3=fully calibrated)
}system_state_t;


/***************************************************************************
*
* Global Variables
*
***************************************************************************/

// Holds the setpoint data structure with current setpoints
extern setpoint_t setpoint;

// Holds the system state structure with current system statesystem_state_t sstate;
extern system_state_t sstate;

// Holds the calibration values for the MS5837 pressure sensor
extern pressure_calib_t pressure_calib;

// Holds the latest pressure value from the MS5837 pressure sensor
extern ms5837_t ms5837;

// Create structure for storing IMU data
extern bno055_t bno055;

// Holds the latest temperature value from the DS18B20 temperature sensor
extern ds18b20_t ds18b20;

// Holds the constants and latest errors of the yaw pid controller
extern pid_data_t yaw_pid;

// Holds the constants and latest errors of the depth pid controller
extern pid_data_t depth_pid;

// Motor channels
extern int motor_channels[];

// Ignoring sstate
extern float depth;


/******************************************************************************
*
* Function Prototypes
*
******************************************************************************/

// Functions for setting motor PWM with PCA9685
int initialize_motors(int channels[3], float freq);
int saturate_number(float* val, float min, float max);
int set_motor(int motor_num, float speed);

// Yaw controller declaration function
float yaw_controller(bno055_t bno055, pid_data_t yaw_pid);

// Functions for reading MS5837 Pressure Sensor
pressure_calib_t init_ms5837(); 			   // initialize ms5837
ms5837_t ms5837_read(pressure_calib_t arg_in); // read values from ms5837

// Functions for reading BNO055 IMU
void start_Py_bno055(void); // start Python background process
bno055_t bno055_read(void); // read values from bno055

// Functions for reading DS18B20 temperature sensor
void start_Py_ds18b20(void); 	// start Python background process
ds18b20_t ds18b20_read(void);	// read values from ds18b20

// Startup functions
int scripps_auv_init(void);

// Cleanup and shutdown
void ctrl_c(int signo); // signal catcher
int cleanup_auv();		// call at the very end of main()

#endif
