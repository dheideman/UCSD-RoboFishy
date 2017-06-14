#include "Mapper.h"

/******************************************************************************
 * Safety Thread
 *
 * Shuts down AUV if vehicle goes belows 10m, temperature gets too high, or
 * water intrusion is detected
 *****************************************************************************/
void *safety_thread(void* arg)
{
	printf("Safety Thread Started\n");

	// open a log to record reasons for shutting down
	std::ofstream logFile;
	logFile.open("safety.log");

	// Leak detection pins
	pinMode(LEAKPIN, INPUT);					// set LEAKPIN as an INPUT
	pinMode(LEAKPOWERPIN, OUTPUT);		// set as output to provide Vcc
	digitalWrite(LEAKPOWERPIN, HIGH);	// write high to provide Vcc

	while( substate.mode != STOPPED )
	{
		// Check if depth threshold has been exceeded
		if( ms5837.depth > STOP_DEPTH )
		{
			substate.mode = STOPPED;
			logFile << "Shut down due to max depth being reached\n";
			logFile << "Stop depth: STOP_DEPTH\n";
			logFile << "Current depth: " << ms5837.depth << "\n";
			printf("\nWe're too deep! Shutting down...\n");
			continue;
		}
		// Check battery compartment temperature
		float _temp = read_temp_fifo();
		if( _temp > STOP_TEMP )
		{
			substate.mode = STOPPED;
			logFile << "Shut down due to max battery temp being reached\n";
			logFile << "Stop temp: STOP_TEMP\n";
			logFile << "Current temp: " << _temp << "\n";
			printf("\nMax battery temp reached: ( %5.2f C)! Shutting down...\n",_temp);
			continue;
		}

		// check pi cpu tem
		// temp is multiplied by 1000 in raspbian OS
		float _cpu_temp = read_cpu_temp();
		if (_cpu_temp > 80000) {
			logFile << "Shut down due to max cpu temp being reached\n";
			logFile << "Stop temp: 80 C\n";
			logFile << "Current temp: " << _cpu_temp << "\n";
			printf("CPU is above 80 C. Shutting down...\n");
			substate.mode = STOPPED;
		}
		// Check for leak
		if( digitalRead(LEAKPIN) == HIGH )
		{
			substate.mode = STOPPED;
			logFile << "Shut down due to leak\n";
			printf("\nLEAK DETECTED! Shutting down...\n");
			continue;
		}

		// Check IMU accelerometer for too much pitch
		if(  (float)fabs(substate.imu.pitch) > PITCH_LIMIT )
		{
		  substate.mode = STOPPED;
			logFile << "Shut down due to excessive pitch (PITCH_LIMIT deg)\n";
			char accel[100];
			sprintf(accel, "Pitch: %5.2f  Roll: %5.2f\n",
				substate.imu.pitch, substate.imu.roll);
		}

		// Check IMU accelerometer for too much roll
		if( (float)fabs(substate.imu.roll) > ROLL_LIMIT )
		{
		  substate.mode = STOPPED;
			logFile << "Shut down due to excessive roll (ROLL_LIMIT deg)\n";
			char accel[100];
			sprintf(accel, "Pitch: %5.2f  Roll: %5.2f\n",
				substate.imu.pitch, substate.imu.roll);
		}

		// Check IMU accelerometer for collision (1+ g detected)
		if(  (float)fabs(substate.imu.x_acc) > 1.0*GRAVITY
			|| (float)fabs(substate.imu.y_acc) > 1.0*GRAVITY
			|| (float)fabs(substate.imu.z_acc) > 2.0*GRAVITY )
		{
			substate.mode = STOPPED;
			logFile << "Shut down due to excessive acceleration (1 g)\n";
			char accel[100];
			sprintf(accel, "X Acc: %5.2f  Y Acc: %5.2f  Z Acc: %5.2f\n",
				substate.imu.x_acc, substate.imu.y_acc, substate.imu.z_acc);
			logFile << accel;
			printf("\nCollision detected. Shutting down...");
			continue;
		}

		// Sleep a bit
		auv_msleep(1000/SAFETY_RATE);
	}
	// close log file
	logFile.close();

	// Exit thread
  pthread_exit(NULL);
}//*/
