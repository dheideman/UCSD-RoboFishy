//Main script for the 2017 RoboFishy Scripps AUV

#include "Libraries.h"

// Sampling Values //
#define SAMPLE_RATE 200 // sample rate of main control loop (Hz)


// Controller Gains //

// Pitch Controller
#define KP_PITCH 0
#define KI_PITCH 0
#define KD_PITCH 0

// Yaw Controller
#define KP_YAW 0
#define KI_YAW 0
#define KD_YAW 0

// Depth Controller
#define KP_DEPTH 0
#define KI_DEPTH 0
#define KD_DEPTH 0

// Saturation Constants //
#define PITCH_SAT 0		// upper limit of pitch controller
#define YAW_SAT 0		// upper limit of yaw controller
#define INT_SAT 0		// upper limit of integral windup
#define DINT_SAT 0		// upper limit of depth integral windup


// Depth start value //
// positive depth axis is pointed downwards
//#define DEPTH_START -50 //starting depth (mm)


// Data Structures

typedef struct setpoint_t
{
	float roll;			// roll angle (rad)
	float roll_rate;	// roll rate (rad/s)
	float pitch;		// pitch angle (rad)
	float pitch_rate;	// pitch rate (rad/s)
	float yaw;			// yaw angle in (rad)
	float yaw_rate;		// yaw rate (rad/s)
	float depth;		// z component in fixed coordinate system
	float speed;		// speed setpoint
}setpoint_t;

typedef struct system_state_t
{
	float roll;						// current roll angle (rad)
	float pitch[2];					// current pitch angle (rad) 0: current value 1: last value
	float yaw[2];					// current yaw angle (rad) 0: current value 1: last value
	//float last_yaw;				// previous value for crossover detection
	float depth[2];					// depth estimate (m)
	float fdepth[2];				// filtered depth estimate (m)
	
	float p[2];						// first derivative of roll (rad/s)
	float q[2];						// first derivative of pitch (rad/s)
	float r[2];						// first derivative of yaw (rad/s)
	float ddepth;					// first derivative of depth (m/s)

	float sum_error_pitch;			// sum of past pitch errors
	float sum_error_depth;			// sum of past pitch errors
	//float dYaw_err; 			// current and previous roll error
	//float dPitch_err;			// current pitch error
	//float roll_err;  			 	// current  roll error
	//float depth_err; 			// current and previous depth error
	
	int sys;						// system calibrations status (0 or 1)
	int gyro;						// gyro calibrations status (0 or 1)
	int accel;						// accelerometer calibrations status (0 or 1)
	int mag;						// magnetometer calibrations status (0 or 1)
	
	float control_u[4];				// control outputs  depth,roll,pitch,yaw
	float esc_out[4];				// normalized (0-1) outputs to motors
	int num_yaw_spins; 				// remember number of spins around Z
	float imu_roll_on_takeoff;		// raw roll value read on takeoff
}system_state_t;


// Global Variables //
//drive_mode_t drive_mode; 			// holds the current drive mode
//loop_mode_t loop_mode;
//cont_mode_t cont_mode; 				// holds the current controller mode
setpoint_t setpoint; 				// holds the setpoint data structure with current setpoints
system_state_t sstate; 				// holds the system state structure with current system state
bno055_t bno055; 					// holds the latest data values from the BNO055
calib_t calib; 						// holds the calibration values for the MS5837 pressure sensor
ms5837_t ms5837; 					// holds the latest pressure value from the MS5837 pressure sensor
int i; 								//counting integer
//int motor_channels[]  = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4}; // motor channels 
float mix_matrix[4][4] = \
		   {{1, -1, 1,-1}, // Roll
			{ -1, 1,1,-1}, // Pitch
			{1, 1,-1, -1}, // Yaw
			{ 1, 1, 1, 1}}; // Thrust


// Declare threads //
PI_THREAD (trajectory_thread); 		// thread for defining the setpoints (Undetermined rate up to user keep below 2 Hz)
PI_THREAD (navigation_thread); 		// thread for running the control system (200 Hz)
PI_THREAD (depth_thread); 			// thread for measuring the pressure of the system (20 Hz)
PI_THREAD (logging_thread); 		// thread for recording data (10 Hz)

// Declare functions //
int init_controller();				// initialize all values to 0
int mix_controls(float r, float p, float y, float t, float* esc, int rotors);	// combines control inputs into effective control inputs


int main()
{
	/*
	if(scripps_auv_init()<0){
		return -1;
	}
	printf("\nInitialization complete\n");
	set_state(UNINITIALIZED);
	drive_mode = DRIVE_OFF;
	cont_mode = NAVIGATION;
	loop_mode = OUTER;
	//loop_mode = INNER; // use this for pitch control and no depth control

	// start all threads w/ error checking
	if (piThreadCreate (depth_thread) != 0)
	{
		printf ("\nFailed to start depth thread\n");
	}
	if (piThreadCreate (trajectory_thread) != 0)
	{
		printf ("\nFailed to start trajectory thread\n");
	}
	if (piThreadCreate (navigation_thread) != 0)
	{
		printf ("\nFailed to start navigation thread\n");
	}
	if (piThreadCreate (logging_thread) != 0)
	{
		printf ("\nFailed to start logging thread\n");
	}
	
	while(get_state()!=EXITING)
	{
		usleep(100000);
	}
	cleanup_auv();*/




	int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);	//initialize PCA9685 board
	if (fd < 0)
	{
		printf("Error in setup\n");
		return fd;
	}
	pca9685PWMReset(fd);	//Resets output of the board
	int active=1;
	while(active){
	pwmWrite(PIN_BASE, calcTicks(0,HERTZ));	//send input signal that is low enough to reach the 
	delay(5000);
	active=0;
	}				//"neutral" or power-off area in order 
	printf("%s\n","Past setup" );							//to arm the ESC (long beep); green LED on ESC
								//will light up
	//start_Py_bno055();
	//start_Py_ms5837();
	delay(1000); //Delay is so that the IMU can initialize and run bn055_read.py
	while (1){	 //before the code below tries to read it and seg faults.
		
		// read imu values
		/*bno055 = bno055_read();
		float new_yaw = bno055.yaw+sstate.num_yaw_spins*360;
		sstate.roll = bno055.pitch; // intentionally reversed
		sstate.pitch[0] = bno055.roll; // intentionally reversed
		sstate.p[0] = bno055.p;
		sstate.q[0] = bno055.q;
		sstate.r[0] = bno055.r;
		sstate.sys= bno055.sys;
		sstate.gyro = bno055.gyro;
		sstate.accel = bno055.accel;
		sstate.mag = bno055.mag;
		delay(100);*/
	
		//Set Motors to desired PWM Outputs
		pwmWrite (PIN_BASE, 1000);	
		pwmWrite(PIN_BASE+1, 1900);	//set motor to desired PWM output (4000 is the max value the Afro ESCs accept)
		


		//start_Py_ms5837();

		//while(1)
		//{
			

			// get pressure value
			//printf("%s\n","Just before while loop" );
			/*calib = init_ms5837();
			ms5837 = ms5837_read(calib);
			printf("%f\n", ms5837.pressure);
			//printf("%f\n", ms5837);
			delay(100); */
	}


	return 0;
}

/*
PI_THREAD (logging_thread)
{
	while(get_state()!=EXITING){
		FILE *fd = fopen("log.txt", "a");
		char buffer[100] = {0};
		// add logging values to the next line
		sprintf(buffer, "%f %f %f %f %i %i %i %i %f %f %f %f\n",sstate.roll, sstate.pitch[0], sstate.yaw[0], sstate.depth[0],sstate.x[0],
		sstate.y[0], sstate.radius[0], setpoint.x - sstate.x[0], sstate.esc_out[0], sstate.esc_out[1], sstate.esc_out[2], sstate.esc_out[3]);
		fputs(buffer, fd);
		fclose(fd);
		//sleep for 100 ms
		
		usleep(100000);
	}
	return 0;
}
*/



int mix_controls(float r, float p, float y, float t, float* esc, int rotors)
{
	int i = 0;
	// sum control inputs
	for(i=0; i<rotors; i++){
		esc[i]=0;
		esc[i]+=r*mix_matrix[0][i];
		esc[i]+=p*mix_matrix[1][i];
		esc[i]+=y*mix_matrix[2][i];
		esc[i]+=t*mix_matrix[3][i];			
	}
	return 0;
}

int init_controller()
{
	sstate.yaw[0] = 0;		// initialize current yaw angle
	sstate.yaw[1] = 0;		// initialize last value of yaw angle
	sstate.pitch[0] = 0;	// initialize current pitch angle
	sstate.pitch[1] = 0;	// initialize last value of pitch angle
	sstate.r[0] = 0;		// 
	sstate.r[1] = 0;
	sstate.depth[0] = 0;
	sstate.depth[1] = 0;
	sstate.fdepth[0] = 0;	// initialize filtered depth estimate
	sstate.fdepth[1] = 0;	// initialize filtered depth estimate
	return 1;
}



