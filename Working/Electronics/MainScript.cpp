///////////////////////////////////////////////////////////////////////////////////
///////////////Main script for the 2017 RoboFishy Scripps AUV//////////////////////
///////////////////////////////////////////////////////////////////////////////////

#include "Libraries.h"

// Sampling Values //
#define SAMPLE_RATE 200 // sample rate of main control loop (Hz)
#define DT 0.005		// timestep; make sure this is equal to 1/SAMPLE_RATE!

// Conversion Factors //
#define UNITS_KPA 0.1 // converts pressure from mbar to kPa

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Controller Gains ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

// Pitch Controller //
#define KP_PITCH 0
#define KI_PITCH 0
#define KD_PITCH 0

// Yaw Controller //
#define KP_YAW .01
#define KI_YAW 0
#define KD_YAW 1

// Depth Controller //
#define KP_DEPTH 0
#define KI_DEPTH 0
#define KD_DEPTH 0

// Saturation Constants //
#define PITCH_SAT 10	// upper limit of pitch controller
#define YAW_SAT 1		// upper limit of yaw controller
#define INT_SAT 10		// upper limit of integral windup
#define DINT_SAT 10		// upper limit of depth integral windup

// Filter Values //
#define A1 0.3			// for MS5837 pressure sensor
#define A2 0.4			// for MS5837 pressure sensor

// Fluid Densities in kg/m^3 //
#define DENSITY_FRESHWATER 997
#define DENSITY_SALTWATER 1029

// Acceleration Due to Gravity in m/s^2 //
#define GRAVITY 9.81

// Depth start value //
#define DEPTH_START -50 //starting depth (mm)

///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Data Structures ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

typedef enum drive_mode_t
{
	DRIVE_OFF,
	DRIVE_ON,
}drive_mode_t; // contains the drive modes

typedef enum loop_mode_t
{
	INNER,
	OUTER,
}loop_mode_t;	// contains which loop the code is in

typedef enum cont_mode_t
{
	NAVIGATION,
}cont_mode_t;	// contains the controller mode

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

	float sum_error_pitch;		// sum of past pitch errors
	float sum_error_yaw;		// sum of past yaw errors
	float sum_error_depth;		// sum of past pitch errors

	int sys;					// system calibrations status (0=uncalibrated, 3=fully calibrated)
	int gyro;					// gyro calibrations status (0=uncalibrated, 3=fully calibrated)
	int accel;					// accelerometer calibrations status (0=uncalibrated, 3=fully calibrated)
	int mag;					// magnetometer calibrations status (0=uncalibrated, 3=fully calibrated)

	float control_u[4];			// control outputs: depth,roll,pitch,yaw
	float esc_out;				// control output to motors
	//float esc_out[4];			// normalized (0-1) outputs to motors
	int num_yaw_spins; 			// remember number of spins around Z-axis
}system_state_t;


///////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Global Variables /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

drive_mode_t drive_mode; 			// holds the current drive mode
loop_mode_t loop_mode;				// holds the current loop mode
cont_mode_t cont_mode; 				// holds the current controller mode
setpoint_t setpoint; 				// holds the setpoint data structure with current setpoints
system_state_t sstate; 				// holds the system state structure with current system state
bno055_t bno055; 					// holds the latest data values from the BNO055
calib_t calib; 						// holds the calibration values for the MS5837 pressure sensor
ms5837_t ms5837; 					// holds the latest pressure value from the MS5837 pressure sensor
ds18b20_t ds18b20;					// holds the latest temperature value from the DS18B20 temperature sensor
int i; 								//counting integer
int motor_channels[]  = {CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4}; // motor channels 
float mix_matrix[4][4] = \
		   {{1, -1, 1,-1}, // Roll
			{ -1, 1,1,-1}, // Pitch
			{1, -1,-1, -1}, // Yaw
			{ 1, 1, 1, 1}}; // Thrust


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Declare threads /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

PI_THREAD (trajectory_thread); 		// thread for defining the setpoints (Undetermined rate up to user keep below 2 Hz)
PI_THREAD (navigation_thread); 		// thread for running the control system (200 Hz)
PI_THREAD (depth_thread); 			// thread for measuring the distance from the bottom (20 Hz)
PI_THREAD (safety_thread);			// thread for ensuring AUV doesn't travel past 10m and cutting power if housing gets too hot
PI_THREAD (vision_thread);			// thread for relative positioning via RaspiCam 
PI_THREAD (logging_thread); 		// thread for recording data (10 Hz)


///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Declare functions ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

int init_controller();															// initialize all values to 0
int mix_controls(float r, float p, float y, float t, float* esc, int rotors);	// combines control inputs into effective control inputs


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Main Function ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

int main()
{
	// Check if AUV is initialized correctly //
	if(scripps_auv_init()<0)
	{
		return -1;
	}
	printf("\nAll components are initializated\n");
	set_state(UNINITIALIZED);
	drive_mode = DRIVE_OFF;
	cont_mode = NAVIGATION;
	//loop_mode = OUTER;
	loop_mode = INNER; // use this for yaw control and no depth control

	// start all threads w/ error checking //
	/*if (piThreadCreate (depth_thread) != 0)
	{
		printf ("\nFailed to start depth thread\n");
	}*/
	/*if (piThreadCreate (trajectory_thread) != 0)
	{
		printf ("\nFailed to start trajectory thread\n");
	}*/
	if (piThreadCreate (navigation_thread) != 0)
	{
		printf ("\nFailed to start navigation thread\n");
	}
	/*if (piThreadCreate (logging_thread) != 0)
	{
		printf ("\nFailed to start logging thread\n");
	}
	*/
	/*if (piThreadCreate (temperature_thread) != 0)
	{
		printf ("\nFailed to start temperature thread\n");
	}*/
	while(get_state()!=EXITING)
	{
		usleep(100000);
	}
	cleanup_auv();
	return 0;
}


///////////////////////////////////////////////////////////////////////////////////
///// Depth Thread for Recording Depth & Determining If AUV is in Water or Not ////
///////////////////////////////////////////////////////////////////////////////////
/*
PI_THREAD (depth_thread)
{
	
}
*/


///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Safety Thread ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
/*PI_THREAD (safety_thread)
{
	// read depth values from MS5837 pressure sensor //
	while(get_state()!=EXITING)
	{
		// get pressure value //
		calib = init_ms5837();
		ms5837 = ms5837_read(calib);
		//sstate.depth[0] = (ms5837.pressure-1013)*10.197-88.8;
		sstate.depth[0] = (ms5837.pressure*UNITS_KPA-101.325)/
			(DENSITY_FRESHWATER*GRAVITY)*0.000001;		// current depth in mm
		printf("%f\n", sstate.depth[0]);

		// check if depth is above start threshold //
		//sstate.fdepth[0] = A1*(sstate.depth[0]+sstate.depth[1])+A2*sstate.fdepth[1]; 
		//if(sstate.fdepth[0]>DEPTH_START)
		if(sstate.depth[0]<DEPTH_START)
		{
			drive_mode = DRIVE_ON;
			printf("%s\n", "DRIVE_ON");
		}
		else
		{
			drive_mode = DRIVE_OFF;
			printf("%s\n", "DRIVE_OFF");
		}
		
		// set current depth values as old values //
		sstate.depth[1] = sstate.depth[0];
		sstate.fdepth[1] = sstate.fdepth[0];
		
		//sleep for 50 ms //
		usleep(50000);
	}
	return 0;

	// read temperature values from DS18B20 temperature sensor //
					//ds18b20 = ds18b20_read();	// temperature in deg C
					//printf("Temperature: %f", ds18b20.temperature);	
					/*if(ds18b20.temperature>60)	
					{
						for( i=0; i<4; i++ )
						{
							pwmWrite(PIN_BASE+i, 2674);	// turn motors off if temperature gets too high
						}
					}*/
//}

///////////////////////////////////////////////////////////////////////////////////
/////////////////// Navigation Thread for Main Control Loop ///////////////////////
///////////////////////////////////////////////////////////////////////////////////

PI_THREAD (navigation_thread)
{
	static float u[4];	// normalized roll, pitch, yaw, throttle, components
	initialize_motors(motor_channels, HERTZ);
	//static float new_esc[4];
	float output_port;		// port motor output
	float output_starboard;	// starboard motor output
	printf("\n");
	init_controller();
	//delay(1000); // Delay is so that the IMU can initialize and run bn055_read.py
	
	while(get_state()!=EXITING)
	{
		// set motors to constant PWM output for straight line //
		pwmWrite (PIN_BASE+motor_channels[0], 2106.4);	// set motor to 20%
		pwmWrite (PIN_BASE+motor_channels[1], 3819.6);	// set motor to 20%

		// read IMU values
		bno055 = bno055_read();
		//float new_yaw = bno055.yaw+sstate.num_yaw_spins*360;
		sstate.yaw[0] = bno055.yaw;		
		sstate.roll = bno055.pitch; // intentionally reversed 
		sstate.pitch[0] = bno055.roll; // intentionally reversed
		sstate.p[0] = bno055.p;
		sstate.q[0] = bno055.q;
		sstate.r[0] = bno055.r;
		sstate.sys= bno055.sys;
		sstate.gyro = bno055.gyro;
		sstate.accel = bno055.accel;
		sstate.mag = bno055.mag;
		printf("\nYaw: %f Roll: %f Pitch: %f p: %f q: %f r: %f Sys: %i Gyro: %i Accel: %i Mag: %i\n ", 
		sstate.yaw[0],bno055.pitch,bno055.roll, bno055.p,bno055.q,bno055.r,
		bno055.sys,bno055.gyro,bno055.accel,bno055.mag);
		//delay(1000);		// wait 1 sec until next read of IMU values
		//delay(100);		// wait 0.1 sec until next read of IMU values

		///////////////////////////////////////////////////////////////////
		///////////// Sanity Test: Check if yaw control works /////////////
		///////////////////////////////////////////////////////////////////

		// switch case to check if the vehicle is in the water
		//switch (drive_mode)
		//{
			/*case DRIVE_OFF:
				for( i = 0; i <4; i++)
				{
					pwmWrite (PIN_BASE+motor_channels[i], 2674);	// set motor output to 0
				}
				break;
			case DRIVE_ON:
				switch (cont_mode)
				{
					case NAVIGATION:
				*/
					// setpoints //
					setpoint.yaw = 0;

					// control output //
					if(sstate.yaw[0]<180)	// AUV is pointed right
					{
						// u[2] is negative
						u[2] = KP_YAW*(setpoint.yaw-sstate.yaw[0]); //+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller
					}
					else    // AUV is pointed left
					{
						// u[2] is positive
						u[2] = KP_YAW*(setpoint.yaw-(sstate.yaw[0]-360)); //+ KD_YAW*(sstate.yaw[0]-sstate.yaw[1])/DT; // yaw controller	
					}

					// saturate yaw controller //
					if(u[2]>YAW_SAT)
					{
						u[2]=YAW_SAT;
					}
					else if(u[2]<-YAW_SAT)
					{
						u[2]=-YAW_SAT;
					}

					// mix controls //
					printf("u[2]: %f\n", u[2]);
					/*if(mix_controls(u[0],u[1],u[2],u[3],&new_esc[0],2)<0)
					{
						printf("ERROR, mixing failed\n");
					}
					for(i = 0; i < 2; i++)
					{
						if(new_esc[i]>1.0)
						{
							new_esc[i]=1.0;
						}
						else if(new_esc[i]<-1.0)
						{
							new_esc[i]=-1.0;
						}
					}*/


					// ESC outputs //
					sstate.esc_out = u[2];
					//sstate.esc_out[0] = new_esc[0];		// port motor (CCW)
					//sstate.esc_out[1] = new_esc[1];		// starboard motor (CW)

					// print ESC values
					//printf("ESC1: %f ESC2: %f \n ", sstate.esc_out[0],sstate.esc_out[1]);

					// saturate motor output values //
					if(sstate.yaw[0]>180)	// port motor (CCW)
					{
						output_port = -26.18*100*sstate.esc_out+2630;
						if(output_port<(2630-(0.2*(2630-12))))	// set motor output at 20% of max for testing purposes (20% = 2106.4)
						{
							output_port = 2630-(0.2*(2630-12));		// for testing purposes
							printf("Port PWM Output1: %f\n", output_port);
						}
				
						output_starboard = 3155.4-(2630-output_port)/(2630-12)*(4905-2718);	// starboard motor output = base 20% minus percentage that port motor increased by	
						if(output_starboard<(2718+0.1*(4905-2718)))
						{
							output_starboard =  2718+0.1*(4905-2718);	// set starboard motor output to no less than 10%
						}		

						output_port = output_port-0.2*(2630-12);			// port motor max at 40%
						pwmWrite(PIN_BASE+motor_channels[0], output_port);	// port motor at base 20% + yaw control output
						pwmWrite(PIN_BASE+motor_channels[1], output_starboard);				// starboard motor at base 20%
					}
					else	// starboard motor (CW)
					{
						output_starboard = 13.77*100*-sstate.esc_out+2718;
						if(output_starboard>(2718+(0.2*(4905-2718)))) // set motor output at 20% of max for testing purposes (20% = 3155.4)
						{
							output_starboard = 2718+(0.2*(4905-2718));	// for testing purposes
							//output_starboard = 4095;
							printf("Starboard PWM Output1: %f\n", output_starboard);
						}

						output_port = 2106.4 - (output_starboard-2718)/(4905-2718)*(2630-12);	// port motor output = base 20% minus percentage that starboard motor increased by
						if(output_port>(2630-(0.1*(2630-12))))
						{
							output_port = 2630-(0.1*(2630-12));		// set port motor output to no less than 10%
						}

						output_starboard = output_starboard+0.2*(4905-2718);	// starboard motor max at 40%
						pwmWrite(PIN_BASE+motor_channels[1], output_starboard);	//	starboard motor output = base 20% + yaw control output
						pwmWrite(PIN_BASE+motor_channels[0], output_port);				// port motor at base 20%
					}
					
					// print motor PWM outputs //
					printf("Port PWM Output2: %f Starboard PWM Output2: %f\n", 
						output_port, output_starboard);

					// set old values to current values //
					sstate.yaw[1] = sstate.yaw[0];
					sstate.pitch[1] = sstate.pitch[0];
					sstate.p[1] = sstate.p[0];
					sstate.q[1] = sstate.q[0];
					sstate.r[1] = sstate.r[0];

					delay(250);		// wait 0.25 sec until next read of IMU values

					// sleep for 5 ms //
					//usleep(5000);
				//}
		//}	
	}
	return 0;
}




///////////////////////////////////////////////////////////////////////////////////
/////////////////////// Logging Thread for Recording Data /////////////////////////
///////////////////////////////////////////////////////////////////////////////////
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
	// sum control inputs //
	for(i=0; i<rotors; i++)
	{
		esc[i]=0;						// initialize esc output
		esc[i]+=r*mix_matrix[0][i];		// roll correction
		esc[i]+=p*mix_matrix[1][i];		// pitch correction
		esc[i]+=y*mix_matrix[2][i];		// yaw correction
		esc[i]+=t*mix_matrix[3][i];		// thrust correction
	}
	return 0;
}

int init_controller()
{
	sstate.yaw[0] = 0;		// initialize current yaw angle
	sstate.yaw[1] = 0;		// initialize last value of yaw angle
	sstate.pitch[0] = 0;	// initialize current pitch angle
	sstate.pitch[1] = 0;	// initialize last value of pitch angle
	sstate.r[0] = 0;		// initialize current roll rate
	sstate.r[1] = 0;		// initialize last value of roll rate
	sstate.depth[0] = 0;	// initialize current depth
	sstate.depth[1] = 0;	// initialize last value of depth
	sstate.fdepth[0] = 0;	// initialize filtered depth estimate
	sstate.fdepth[1] = 0;	// initialize filtered depth estimate
	sstate.speed = 0;		// initialize speed
	return 1;
}
