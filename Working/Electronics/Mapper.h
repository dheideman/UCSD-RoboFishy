#ifndef MAPPER_H
#define MAPPER_H

#include <pca9685.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cstdio>
#include <stdlib.h>
#include <Python.h>
#include <time.h>
#include <signal.h>		// capture control-c

// Core Module
#include "../../Modules/Core/Core.h"


#define DEBUG

#define PCA9685_ADDR 0x40
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 400
#define PCA9685_ADDR 0x40
#define CHANNEL_1 0		// port motor
#define CHANNEL_2 1		// starboard motor
#define CHANNEL_3 2		// vertical motor

#define MOTOR_0 2674    // motor output is 0

// Motor Constants
#define MOTOR_DEADZONE 0.05   // 5 percent
#define PWM_LOW_LIMIT  1940   // PWM value
#define PWM_HIGH_LIMIT 3354   // PWM value
#define PWM_ZERO_VALUE 2647   // PWM value

// Protection Constants
#define DEPTH_STOP 2000	// threshold depth (mm)
#define TEMP_STOP 50	// deg C


/***************************************************************************
* Create Structs
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

// Struct for PID Controllers
typedef struct pid_data_t
{
	float kp, ki, kd;
	float perr, ierr, derr;
	float setpoint;
	float output;
	float old;
	float sat;
	float dt;
	float isat;
}pid_data_t;

// Struct for setpoints
/*typedef struct setpoint_t
{
	float yaw;				// yaw angle in (rad)
	float yaw_rate;			// yaw rate (rad/s)
	float depth;			// z component in fixed coordinate system
	float speed;			// speed setpoint
}setpoint_t;*/



/***************************************************************************
* Global Variables
***************************************************************************/

// Holds the setpoint data structure with current setpoints
//extern setpoint_t setpoint;

// Holds the calibration values for the MS5837 pressure sensor
extern pressure_calib_t pressure_calib;

// Holds the latest pressure value from the MS5837 pressure sensor
extern ms5837_t ms5837;

// Holds the latest temperature value from the DS18B20 temperature sensor
extern float ds18b20;

// Holds the constants and latest errors of the yaw pid controller
extern pid_data_t yaw_pid;

// Holds the constants and latest errors of the depth pid controller
extern pid_data_t depth_pid;

// Motor channels
extern int motor_channels[];

// Ignoring sstate
extern float depth;



/******************************************************************************
* Function Prototypes
******************************************************************************/

// Functions for setting motor PWM with PCA9685
int initialize_motors(int channels[3], float freq);
int saturate_number(float* val, float min, float max);
int set_motor(int motor_num, float speed);

// Yaw controller declaration function
float marchPID(pid_data_t controller, float input);

// Functions for reading MS5837 Pressure Sensor
void start_read_pressure(void);			// start Python background process
pressure_calib_t init_pressure_sensor(void);	// initialize ms5837
// read values from ms5837
// need to pass calibration values to it
ms5837_t read_pressure(pressure_calib_t pressure_calib);

// Functions for reading BNO055 IMU
void start_read_imu(void); 					// start Python background process
imu_t read_imu_fifo(void); 			// read values from bno055

// Functions for reading DS18B20 temperature sensor
void start_read_temp(void); 				// start Python background process
float read_temp_fifo(void);			// read values from ds18b20

// Startup functions
int initialize_sensors(void);

// Cleanup and shutdown
void ctrl_c(int signo); 		// signal catcher
int cleanup_auv();					// call at the very end of main()

#endif
