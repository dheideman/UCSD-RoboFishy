#ifndef MAPPER_H
#define MAPPER_H

#include <pca9685.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cstdio>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>		// capture ctrl+c

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// Core Module
#include "../Modules/Core/Core.h"
#include "../Modules/Camera/Camera.h"
#include "../Modules/RangeFinder/RangeFinder.h"
#include "../Modules/Odometry/Odometry.h"

//#define DEBUG
#define RASPBERRY_PI

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
#define MOTOR_DEADZONE 0.03   // 3 percent
#define PWM_LOW_LIMIT  1940   // PWM value
#define PWM_HIGH_LIMIT 3354   // PWM value
#define PWM_ZERO_VALUE 2647   // PWM value

// Protection Constants
#define STOP_DEPTH 2	// threshold depth (m)
#define STOP_TEMP 50	// deg C

// Sampling Values
#define SAMPLE_RATE 50  // sample rate of main control loop (Hz)
#define DT 0.02 				// timestep; make sure this is equal to 1/SAMPLE_RATE!

// Print/Logging Rate
#define LOG_RATE      10  // Hz

// Safety Thread Rate
#define SAFETY_RATE   1  // Hz

// Conversion Factors
#define UNITS_KPA 0.1		 // converts pressure from mbar to kPa

// Yaw Controller
#define KP_YAW 0.001
#define KI_YAW 0
#define KD_YAW 0

// Depth Controller
#define KP_DEPTH 0.1
#define KI_DEPTH 0
#define KD_DEPTH 0

// Saturation Constants
#define YAW_SAT 1			// upper limit of yaw controller
#define DEPTH_SAT 1		// upper limit of depth controller
#define INT_SAT 10		// upper limit of integral windup
#define DINT_SAT 10		// upper limit of depth integral windup

// Pitch/Roll limits
#define ROLL_LIMIT    1000  // degrees
#define PITCH_LIMIT   1000  // degrees

// Acceleration Due to Gravity in m/s^2
#define GRAVITY 9.81

// Depth Start Value
#define DEPTH_START 0.05 // starting depth (m)

// Stop Timer
#define STOP_TIME 20			// seconds

// Leak Sensor Input and Power Pin
#define LEAKPIN       27	// connected to GPIO 27
#define LEAKPOWERPIN  17  // providing Vcc to leak board

// Time Per Straight Leg of "Path"
#define LEG_TIME    1.5   // seconds
#define DELTA_SETPOINT 45 // degrees

/***************************************************************************
* Create Structs
***************************************************************************/

// Struct for holding MS5837 return values
typedef struct
{
	float depth;
	float water_temp;
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
	float period;
}pid_data_t;

/***************************************************************************
* Global Variables
***************************************************************************/

// Holds the latest pressure value from the MS5837 pressure sensor
extern ms5837_t ms5837;

// Holds the constants and latest errors of the yaw pid controller
extern pid_data_t yaw_pid;

// Holds the constants and latest errors of the depth pid controller
extern pid_data_t depth_pid;

// Motor channels
extern int motor_channels[];

// Holds the latest temperature value from the temperature temperature sensor
extern float batt_temp;

// setmotor intialization
float portmotorspeed = 0;
float starmotorspeed = 0;

// Start time for stop timer
extern struct timeval start, now;


/******************************************************************************
 * Declare Threads
******************************************************************************/

void *navigation_thread(void* arg);
void *log_thread(void* arg);
void *safety_thread(void* arg);
void *userInterface(void* arg);


/******************************************************************************
* Function Prototypes
******************************************************************************/
// Functions for setting motor PWM with PCA9685
int initialize_motors(int channels[3], float freq);
int saturate_number(float* val, float min, float max);
float set_motor(int motor_num, float speed);

// Yaw controller declaration function
float marchPID(pid_data_t controller, float input);

// Functions for reading MS5837 Pressure Sensor
void start_read_pressure(void);			// start Python background process
ms5837_t read_pressure_fifo(void);  // read values from ms5837

// Functions for reading BNO055 IMU
void start_read_imu(void); 					// start Python background process
imu_t read_imu_fifo(void); 					// read values from bno055

// Functions for reading DS18B20 temperature sensor
void start_read_temp(void); 				// start Python background process
float read_temp_fifo(void);					// read values from ds18b20

// Functions for reading ras pi status
float read_cpu_temp(void);

// Startup functions
int initialize_sensors(void);

// Cleanup and shutdown
int cleanup_auv();									// call at the very end of main()

#endif
