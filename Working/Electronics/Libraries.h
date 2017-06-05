#include <pca9685.h>
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


/***************************************************************************
 *
 * Create Structs
 *
***************************************************************************/

// struct for hold ms5837 calibration values
typedef struct
{
	float C1, C2, C3, C4, C5, C6;
}pressure_calib_t;

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

// Program Flow and State Control
typedef enum state_t
{
	UNINITIALIZED,
	RUNNING,
	PAUSED,
	EXITING
}state_t;

//struct for PID Controllers
typedef struct pid_data_t
{
	float kp, ki, kd;
	float p_err, i_err, d_err;
	float setpoint;
	float oldyaw;
}pid_data_t;

typedef struct setpoint_t
{
	float roll;				// roll angle (rad)
	float roll_rate;	// roll rate (rad/s)
	float pitch;			// pitch angle (rad)
	float pitch_rate; // pitch rate (rad/s)
	float yaw;				// yaw angle in (rad)
	float yaw_rate;		// yaw rate (rad/s)
	float depth;			// z component in fixed coordinate system
	float speed;			// speed setpoint
}setpoint_t;

typedef struct system_state_t
{
	float roll;					// current roll angle (rad)
	float pitch[2];			// current pitch angle (rad) 0: current value, 1: last value
	float yaw[2];				// current yaw angle (rad) 0: current value, 1: last value
	float depth[2];			// depth estimate (m)
	float fdepth[2];		// filtered depth estimate (m)
	float speed;				// speed (m/s)

	float p[2];					// first derivative of roll (rad/s)
	float q[2];					// first derivative of pitch (rad/s)
	float r[2];					// first derivative of yaw (rad/s)
	float ddepth;				// first derivative of depth (m/s)

	int sys;		// system calibrations status (0=uncalibrated, 3=fully calibrated)
	int gyro;		// gyro calibrations status (0=uncalibrated, 3=fully calibrated)
	int accel;	// accelerometer calibrations status (0=uncalibrated, 3=fully calibrated)
	int mag;		// magnetometer calibrations status (0=uncalibrated, 3=fully calibrated)

	float control_u[4];			// control outputs: depth,roll,pitch,yaw
	float esc_out;				// control output to motors
	//float esc_out[4];			// normalized (0-1) outputs to motors
	int num_yaw_spins;			// remember number of spins around Z-axis
}system_state_t;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Global Variables /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// holds the setpoint data structure with current setpoints
setpoint_t setpoint;

// holds the system state structure with current system statesystem_state_t sstate;
system_state_t sstate;

// holds the calibration values for the MS5837 pressure sensor
pressure_calib_t pressure_calib;

// holds the latest pressure value from the MS5837 pressure sensor
ms5837_t ms5837;

// create structure for storing IMU data
bno055_t bno055;

// holds the latest temperature value from the DS18B20 temperature sensor
ds18b20_t ds18b20;

// holds the constants and latest errors of the yaw pid controller
pid_data_t yaw_pid;

// holds the constants and latest errors of the depth pid controller
pid_data_t depth_pid;

int motor_channels[]	= {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4}; // motor channels

// Ignoring sstate
float depth = 0;

// Thread attributes for different priorities
pthread_attr_t tattrlow, tattrmed, tattrhigh;


/******************************************************************************
*
* Function Prototypes
*
******************************************************************************/

// System state
enum state_t get_state();
int set_state(enum state_t);

// Functions for setting motor PWM with PCA9685
int calcTicks(float impulseMs, int hertz);
int initialize_motors(int channels[4], float freq);
int saturate_number(float* val, float min, float max);

int set_motors(int motor_num, float speed)

// Functions for Reading MS5837 Pressure Sensor
pressure_calib_t init_ms5837(); // initialize ms5837
ms5837_t ms5837_read(pressure_calib_t arg_in); // read values from ms5837

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
