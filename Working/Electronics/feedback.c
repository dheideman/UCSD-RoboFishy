//Main control script for the Scripps AUV
// Matthew Pearson Team Fish Sticks 2015

#include "scripps_auv.h"

// rate values //
#define SAMPLE_RATE 200 // sample rate of main control loop in Hz
#define DT 0.005 // timestep 1/SAMPLE_RATE these must be the same!

// controller gains //
//pitch depth controller
#define KP_PITCH 0.0005
#define KI_PITCH 0.00005
#define KD_PITCH 0.0001
// yaw
#define KP_YAW 0.01
#define KD_YAW 0.002
// roll
#define KP_ROLL_RATE 0.2
//depth
//#define KP_DEPTH 0.02
//#define KI_DEPTH 0
//#define KD_DEPTH 0.008
//#define KP_DEPTH 0.002
//#define KI_DEPTH 0.0004
//#define KD_DEPTH 0.0002
#define KP_DEPTH 0.005
#define KI_DEPTH 0.001
#define KD_DEPTH 0.001
// x coordinate of frame
//#define KP_X -0.00002
//#define KD_X 0
//#define KP_X 0.00008
//#define KD_X 0.00000
//WORKING
#define KP_X 0.0008
#define KD_X 0.00000

//#define KP_X 0.0005
//#define KD_X 0
//#define KD_X 0.000008
// y coordinate of frame
#define KP_Y 0.002
#define KD_Y 0.0007
// radius
#define KP_RAD 0.0006
#define KD_RAD 0

// WORKING DO NOT TOUCH //
/*#define KP_PITCH 0.004
#define KI_PITCH 0
#define KD_PITCH 0.0002
// yaw
#define KP_YAW 0.01
#define KD_YAW 0.002
//depth
#define KP_DEPTH 0.05
#define KI_DEPTH 0.01
#define KD_DEPTH 0.006
// x coordinate
#define KP_X 0.0008
#define KD_X 0.0000*/

// Saturation Constants
#define PITCH_SAT 0.6
#define INT_SAT 500
#define DINT_SAT 500
#define YAW_SAT 0.30

// Filter Values
#define A1 0.3
#define A2 0.4

// Filter Values
#define B1 0.15
#define B2 0.7

// Depth start value
#define DEPTH_START -80 //starting depth in mm

// Data Structures

typedef enum drive_mode_t{
	DRIVE_OFF,
	DRIVE_ON,
}drive_mode_t; // contains the drive modes

typedef enum loop_mode_t{
	INNER,
	OUTER,
}loop_mode_t;

typedef enum cont_mode_t{
	NAVIGATION,
	VISION,
}cont_mode_t; // contains the controller modes

typedef struct setpoint_t{
	float roll_rate;	// roll rate (rad/s)
	float pitch;			// pitch angle (rad)
	float pitch_rate;	// pitch rate (rad/s)
	float yaw;			// yaw angle in (rad)
	float yaw_rate;	// yaw rate (rad/s)
	float depth;		// z component in fixed coordinate system
	float speed;		// speed setpoint
	int x;					// desired x coordinate of image
	int y; 				// desired y coordinate of image
	int radius;			// desired radius of image
}setpoint_t;

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

// Global Variables
drive_mode_t drive_mode; // holds the current drive mode
loop_mode_t loop_mode;
cont_mode_t cont_mode; // holds the current controller mode
setpoint_t setpoint; // holds the setpoint data structure with current setpoints
system_state_t sstate; // holds the system state structure with current system state
bno055_t bno055; // holds the latest data values from the BNO055
calib_t calib; // holds the calibration values for the MS5837 pressure sensor
ms5837_t ms5837; // holds the latest pressure value from the MS5837 pressure sensor
int i; //counting integer
int motor_channels[]  = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4}; // motor channels 
float mix_matrix[4][4] = \
		   {{1, -1, 1,-1}, // Roll
			{ -1, 1,1,-1}, // Pitch
			{1, 1,-1, -1}, // Yaw
			{ 1, 1, 1, 1}}; // Thrust

// Declare threads
PI_THREAD (trajectory_thread); // thread for defining the setpoints (Undetermined rate up to user keep below 2 Hz)
PI_THREAD (navigation_thread); // thread for running the control system (200 Hz)
PI_THREAD (depth_thread); // thread for measuring the pressure of the system (20 Hz)
PI_THREAD (vision_thread); // thread for reading the vision setpoints and checking if the vision system is active (4 Hz)
PI_THREAD (logging_thread); // thread for recording data (10 Hz)

// Declare functions
int zero_controller();
int init_controller();
int mix_controls(float r, float p, float y, float t, float* esc, int rotors);

int main(){
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
	if (piThreadCreate (depth_thread) != 0){
		printf ("\nFailed to start depth thread\n");
	}
	if (piThreadCreate (trajectory_thread) != 0){
		printf ("\nFailed to start trajectory thread\n");
	}
	if (piThreadCreate (navigation_thread) != 0){
		printf ("\nFailed to start navigation thread\n");
	}
	if (piThreadCreate (vision_thread) != 0){
		printf ("\nFailed to start vision thread\n");
	}
	if (piThreadCreate (logging_thread) != 0){
		printf ("\nFailed to start logging thread\n");
	}
	
	while(get_state()!=EXITING){
		usleep(100000);
	}
	cleanup_auv();
	return 0;
}

// trajectory thread for defining the controller setpoints
PI_THREAD (trajectory_thread){
	int j;
	
	while(get_state()!=EXITING){
		
		switch (cont_mode){
		case NAVIGATION:
			// user input
			//setpoint.pitch = 0;
			/*setpoint.yaw = 0;
			setpoint.speed = 0.6;
			//setpoint.speed = 0;
			setpoint.roll_rate = 0;
			setpoint.depth = 400;
			//setpoint.pitch = 0;*/

			// sound test
			/*setpoint.speed = 0;
			for( j = 0; j <5; j++){
				setpoint.speed = setpoint.speed+0.2;
				sleep(5);
			}
			setpoint.speed = 0.001;
			sleep(10);*/
			
			// figure  8
			setpoint.roll_rate = 0;
			setpoint.depth = 150;
			//setpoint.pitch = 0;
			setpoint.speed = 0.4;
			setpoint.yaw = 45;
			sleep(4);
			setpoint.yaw = 135;
			sleep(2);
			setpoint.yaw = 180;
			sleep(2);
			setpoint.yaw = 225;
			sleep(2);
			setpoint.yaw = 315;
			sleep(8);
			setpoint.yaw = 225;
			sleep(2);
			setpoint.yaw = 180;
			sleep(2);
			setpoint.yaw = 135;
			sleep(2);
			setpoint.yaw = 45;
			sleep(4);
			
			// pitch up and down
			/*setpoint.yaw = 0;
			setpoint.pitch = -10;
			setpoint.roll_rate = 0;
			setpoint.yaw_rate = 0.2;
			setpoint.speed = 0.3;
			sleep(3);
			setpoint.yaw = 0;
			setpoint.pitch = 10;
			setpoint.roll_rate = 0;
			setpoint.yaw_rate = 0.2;
			setpoint.speed = 0.3;
			sleep(3);*/
			
		case VISION:
			setpoint.x = 0;
			setpoint.y = 0;
			setpoint.radius = 100; 
		default:
			break;
		}
	}
	return 0;
}

// navigation thread for main control loop
PI_THREAD (navigation_thread){
	static float u[4];	// normalized roll, pitch, yaw, throttle, components
	//int fd = initialize_motors(motor_channels, HERTZ);
	initialize_motors(motor_channels, HERTZ);
	static float new_esc[4];
	printf("\n");
	init_controller();
	while(get_state()!=EXITING){
	
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
		
		// detect the crossover point at Z = +-PI
		if(new_yaw - sstate.yaw[1] > 344){
			sstate.num_yaw_spins -= 1;
		}
		else if(new_yaw - sstate.yaw[1] < -344){
			sstate.num_yaw_spins += 1;
		}
		sstate.yaw[0] = bno055.yaw + sstate.num_yaw_spins*360;
	
		//printf("\nYaw: %f Roll: %f Pitch: %f p: %f q: %f r: %f Sys: %i Gyro: %i Accel: %i Mag: %i                  \n ", 
		//sstate.yaw[0],bno055.pitch,bno055.roll, bno055.p,bno055.q,bno055.r,
		//bno055.sys,bno055.gyro,bno055.accel,bno055.mag);
		
		// switch case to check if the vehicle is in the water
		switch (drive_mode){
		case DRIVE_OFF:
			for( i = 0; i <4; i++){
				pwmWrite (PIN_BASE+motor_channels[i], calcTicks(0.1, HERTZ));
			}
			break;
		case DRIVE_ON:
			switch (cont_mode){
			case NAVIGATION:
				sstate.sum_error_pitch = setpoint.pitch-sstate.pitch[0] + sstate.sum_error_pitch;
				if(sstate.sum_error_pitch>INT_SAT){
					sstate.sum_error_pitch=INT_SAT;
				}
				else if(sstate.sum_error_pitch<-INT_SAT){
					sstate.sum_error_pitch=-INT_SAT;
				}
			
				//u[0] = -KP_ROLL_RATE*setpoint.roll_rate; // roll controller
				u[0] = 0;
				u[1] = KP_PITCH*(setpoint.pitch-sstate.pitch[0]) +KI_PITCH*(sstate.sum_error_pitch)- KD_PITCH*(sstate.pitch[0]-sstate.pitch[1])/DT; // pitch controller
				//u[1] = 0;
				//u[1] = KP_PITCH*(setpoint.pitch-sstate.pitch[0]) - KD_PITCH*(sstate.pitch[0]-sstate.pitch[1])/DT; // pitch controller
				u[2] = KP_YAW*(setpoint.yaw-sstate.yaw[0]) - KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
				u[3] = setpoint.speed; //speed controller

				//u[1] = 0;
				//u[2] = 0;
				//u[3] = setpoint.speed; //speed controller
				
				// saturate pitch controller
				if(u[1]>PITCH_SAT){
					u[1]=PITCH_SAT;
				}
				else if(u[1]<-PITCH_SAT){
					u[1]=-PITCH_SAT;
				}
				
				// saturate yaw controller
				if(u[2]>YAW_SAT){
					u[2]=YAW_SAT;
				}
				else if(u[2]<-YAW_SAT){
					u[2]=-YAW_SAT;
				}
				
				// print control input values
				//printf("u[0]: %f u[1]: %f u[2]: %f u[3]: %f                  \n ", 
				//u[0],u[1],u[2],u[3]);
				
				// mix controls
				if(mix_controls(u[0],u[1],u[2],u[3],&new_esc[0],4)<0){
					printf("ERROR, mixing failed\n");
				}
				for( i = 0; i <4; i++){
					if(new_esc[i]>1.0){
						new_esc[i]=1.0;
					}
					else if(new_esc[i]<-1){
						new_esc[i]=-1;
					}
				}
				
				// print ESC values
				//printf("ESC1: %f ESC2: %f ESC3: %f ESC4: %f                  \n ", 
				//new_esc[0],new_esc[1],new_esc[2],new_esc[3]);
				//fflush(stdout);
				sstate.esc_out[0] = new_esc[0];
				sstate.esc_out[1] = new_esc[1];
				sstate.esc_out[2] = new_esc[2];
				sstate.esc_out[3] = new_esc[3];
				
				// write pwm values
				for( i = 0; i <4; i++){
					pwmWrite (PIN_BASE+motor_channels[i], calcTicks(new_esc[i], HERTZ));
				}
				break;
			case VISION:
				// do nothing
				break;
			default:
				break;
			}
			// // saturate pitch controller
			// if(u[1]>PITCH_SAT){
				// u[1]=PITCH_SAT;
			// }
			// else if(u[1]<-PITCH_SAT){
				// u[1]=-PITCH_SAT;
			// }
			
			// // saturate yaw controller
			// if(u[2]>YAW_SAT){
				// u[2]=YAW_SAT;
			// }
			// else if(u[2]<-YAW_SAT){
				// u[2]=-YAW_SAT;
			// }
			
			// // print control input values
			// //printf("u[0]: %f u[1]: %f u[2]: %f u[3]: %f                  \n ", 
			// //u[0],u[1],u[2],u[3]);
			
			// // mix controls
			// if(mix_controls(u[0],u[1],u[2],u[3],&new_esc[0],4)<0){
				// printf("ERROR, mixing failed\n");
			// }
			// for( i = 0; i <4; i++){
				// if(new_esc[i]>1.0){
					// new_esc[i]=1.0;
				// }
				// else if(new_esc[i]<-1){
					// new_esc[i]=-1;
				// }
			// }
			
			// // print ESC values
			// //printf("ESC1: %f ESC2: %f ESC3: %f ESC4: %f                  \n ", 
			// //new_esc[0],new_esc[1],new_esc[2],new_esc[3]);
			// //fflush(stdout);
			
			// // write pwm values
			// for( i = 0; i <4; i++){
				// pwmWrite (PIN_BASE+motor_channels[i], calcTicks(new_esc[i], HERTZ));
			// }
			
		default:
			break;
		}
		
		// set old values to current values
		sstate.yaw[1] = sstate.yaw[0];
		sstate.pitch[1] = sstate.pitch[0];
		sstate.p[1] = sstate.p[0];
		sstate.q[1] = sstate.q[0];
		sstate.r[1] = sstate.r[0];
		
		// sleep for 5 ms
		usleep(5000);
	}
	return 0;
}

// depth thread for recording depth and determining if the vehicle is in the water or not
PI_THREAD (depth_thread){
	while(get_state()!=EXITING){
		
		// get pressure value
		calib = init_ms5837();
		ms5837 = ms5837_read(calib);
		sstate.depth[0] = (ms5837.pressure-1013)*10.197-88.8;
		//printf("%f\n", sstate.depth);
		// check if depth is above start threshold
		sstate.fdepth[0] = A1*(sstate.depth[0]+sstate.depth[1])+A2*sstate.fdepth[1];
		if(sstate.fdepth[0]>DEPTH_START){
			drive_mode = DRIVE_ON;
		}
		else {
			drive_mode = DRIVE_OFF;
		}
		
		// staturation for depth integral windup
		sstate.sum_error_depth = setpoint.depth-sstate.depth[0] + sstate.sum_error_depth;
				if(sstate.sum_error_depth>DINT_SAT){
					sstate.sum_error_depth=DINT_SAT;
				}
				else if(sstate.sum_error_depth<-DINT_SAT){
					sstate.sum_error_depth=-DINT_SAT;
				}
		
		if(setpoint.speed>0 &&  loop_mode == OUTER){
			
			//setpoint.pitch = KP_DEPTH*(setpoint.depth-sstate.depth[0]) - KD_DEPTH*(sstate.depth[0]-sstate.depth[1])/0.05; // pitch controller
			setpoint.pitch = KP_DEPTH*(setpoint.depth-sstate.depth[0]) +KI_DEPTH*(sstate.sum_error_depth) - KD_DEPTH*(sstate.depth[0]-sstate.depth[1])/0.05; // pitch controller
		}
		
		sstate.depth[1] = sstate.depth[0];
		sstate.fdepth[1] = sstate.fdepth[0];
		
		//sleep for 50 ms
		usleep(50000);
	}
	return 0;
}

PI_THREAD (logging_thread){
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

// vision thread for checking if the target is in range and grabing the pixel data
PI_THREAD (vision_thread){

	static float u[4];	// normalized roll, pitch, yaw, throttle, components
	static float new_esc[4];
	
	while(get_state()!=EXITING){
		
		// get pixel values
		char buf[1000];
		FILE *fd = fopen( "/tmp/feedback_data.fifo", "r");
		fgets(buf,1000,fd);
		fclose(fd);
		sscanf(buf,"%i %i %i", &sstate.x[0],&sstate.y[0],&sstate.radius[0]);
		//printf("%f\n", sstate.radius);
		
		// for navigation mode use NAVIGATION for if and else
		// for visual feedback mode use VISION for if and else
		// for switching between the two, use NAVIGATION for if and VISION for else
		
		// check if the image is in range
		if(sstate.radius[0]==3000){
			cont_mode = NAVIGATION;
			//cont_mode = VISION;
		}
		else {
			//cont_mode = VISION;
			cont_mode = NAVIGATION;
		}
		
		switch (drive_mode){
		case DRIVE_OFF:
			for( i = 0; i <4; i++){
				pwmWrite (PIN_BASE+motor_channels[i], calcTicks(0.1, HERTZ));
			}
			break;
		case DRIVE_ON:
			switch (cont_mode){
			case NAVIGATION:
				break;
			case VISION:
				sstate.fx[0] = B1*(sstate.x[0]+sstate.x[1])+B2*sstate.fx[1];
				//sstate.y[0] = -sstate.y[0];
				u[0] = 0;
				//u[1] = KP_Y*(setpoint.y-sstate.y[0]) - KD_Y*(sstate.y[0]-sstate.y[1])/DT; // x pixel controller
				u[1] = 0;
				u[2] = KP_X*(setpoint.x-sstate.x[0]) - KD_X*(sstate.x[0]-sstate.x[1])/DT; // y pixel controller
				// if(sstate.radius[0] != 3000){
					// u[3] = 0.2 +  KP_RAD*(setpoint.radius-sstate.radius[0]) - KD_RAD*(sstate.radius[0]-sstate.radius[1])/DT; //radius controller
				// }
				// else {
					// u[3] = 0.2;
				// }
				u[3] = 0.15;
				//u[3] = 0.2;
				
				// saturate pitch controller
				if(u[1]>PITCH_SAT){
					u[1]=PITCH_SAT;
				}
				else if(u[1]<-PITCH_SAT){
					u[1]=-PITCH_SAT;
				}
				
				// saturate yaw controller
				if(u[2]>YAW_SAT){
					u[2]=YAW_SAT;
				}
				else if(u[2]<-YAW_SAT){
					u[2]=-YAW_SAT;
				}
				
				// print control input values
				//printf("u[0]: %f u[1]: %f u[2]: %f u[3]: %f                  \n ", 
				//u[0],u[1],u[2],u[3]);
				
				// mix controls
				if(mix_controls(u[0],u[1],u[2],u[3],&new_esc[0],4)<0){
					printf("ERROR, mixing failed\n");
				}
				for( i = 0; i <4; i++){
					if(new_esc[i]>1.0){
						new_esc[i]=1.0;
					}
					else if(new_esc[i]<-1){
						new_esc[i]=-1;
					}
				}
				
				// print ESC values
				//printf("ESC1: %f ESC2: %f ESC3: %f ESC4: %f                  \n ", 
				//new_esc[0],new_esc[1],new_esc[2],new_esc[3]);
				//fflush(stdout);
				
				sstate.esc_out[0] = new_esc[0];
				sstate.esc_out[1] = new_esc[1];
				sstate.esc_out[2] = new_esc[2];
				sstate.esc_out[3] = new_esc[3];
				
				// write pwm values
				for( i = 0; i <4; i++){
					pwmWrite (PIN_BASE+motor_channels[i], calcTicks(new_esc[i], HERTZ));
				}
				
				break;
			default:
				break;
			}
			
			
		default:
			break;
		}
		
		// set old values to current values
		sstate.x[1] = sstate.x[0];
		sstate.fx[1] = sstate.fx[0];
		sstate.y[1] = sstate.y[0];
		sstate.radius[1] = sstate.radius[0];
		
		//sleep for 250 ms
		usleep(250000);
	}
	return 0;
}

int mix_controls(float r, float p, float y, float t, float* esc, int rotors){
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