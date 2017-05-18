#include "Libraries.h"
int init_controller();

typedef struct system_state_t{
	float roll;						// current roll angle (rad)
	float pitch[2];					// current pitch angle (rad) 0: current value 1: last value
	float yaw[2];					// current yaw angle (rad) 0: current value 1: last value
	//float last_yaw;				// previous value for crossover detection
	float depth[2];					// depth estimate (m)
	float fdepth[2];					// filtered depth estimate (m)
	
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
	
	int sys;							// system calibrations status
	int gyro;						// gyro calibrations status
	int accel;						// accelerometer calibrations status
	int mag;						// magnetometer calibrations status
	
	float control_u[4];				// control outputs  depth,roll,pitch,yaw
	float esc_out[4];					// normalized (0-1) outputs to motors
	int num_yaw_spins; 			// remember number of spins around Z
	float imu_roll_on_takeoff;	// raw roll value read on takeoff
	
	int x[2];								// x coordinate of image
	int fx[2];							// filtered x coordinate of image
	int y[2]; 							// y coordinate of image
	int radius[2];						// radius of image
} system_state_t;

//Global Variables
system_state_t sstate; // holds the system state structure with current system state
bno055_t bno055; // holds the latest data values from the BNO055


int main(){
	start_Py_bno055();
	delay(1000); //Delay is so that the IMU can initialize and run bn055_read.py
	while (1){	 //before the code below tries to read it and seg faults.
		
		// read imu values
		bno055 = bno055_read();
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
		printf("%f\n",  bno055.roll);
		delay(100);
	}
}

int init_controller(){
	sstate.yaw[0] = 0;
	sstate.yaw[1] = 0;
	sstate.pitch[0] = 0;
	sstate.pitch[1] = 0;
	sstate.r[0] = 0;
	sstate.r[1] = 0;
	sstate.x[0] = 0;
	sstate.x[1] = 0;
	sstate.y[0] = 0;
	sstate.y[1] = 0;
	sstate.radius[0] = 0;
	sstate.radius[1] = 0;
	sstate.depth[0] = 0;
	sstate.depth[1] = 0;
	sstate.fdepth[0] = 0;
	sstate.fdepth[1] = 0;
	sstate.fx[0] = 0;
	sstate.fx[1] = 0;
	return 1;
}
