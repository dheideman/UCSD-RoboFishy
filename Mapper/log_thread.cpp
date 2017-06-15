#include "Mapper.h"

/******************************************************************************
* Logging Thread
*
* Writes data to screen and to files
******************************************************************************/

void *log_thread(void* arg)
{
  // Start the Kenny Loggings Thread!
	printf("Logging Thread Started\n");

	// open a log to record general data
	std::ofstream genlog;
	genlog.open("mapper.log");

	// open a log to record yaw-related information
	std::ofstream yawlog;
	yawlog.open("yawcontrol.log");

	// Create a string stream to store information for printing and logging
	std::stringstream output;

	while(substate.mode!=STOPPED)
	{
	  // Read pressure values
		ms5837 = read_pressure_fifo();

		// read IMU values from fifo file
		substate.imu = read_imu_fifo();

    // Only print while RUNNING
    if(substate.mode == RUNNING)
    {
      /* Generate general output */

      // Clear string stream
      output.str(std::string());

      // Print yaw
      output << "Yaw: " << substate.imu.yaw << "\t";
      output << "Yaw setpoint: " << yaw_pid.setpoint << std::endl;

      // Print depth
      output << "Depth: " << ms5837.depth << "\t";
      output << "Depth setpoint: " << depth_pid.setpoint << "\t";
      output << "Water temp: " << ms5837.water_temp << " C" << std::endl;

			// Print Range to the Bottom
      output << "Range: " << substate.range << std::endl;

      // Print battery temperature
      output << "Battery temp: " << read_temp_fifo() << " C" << std::endl;

      // Write a newline
      output << std::endl;

      // Write IMU data
/*
      printf("\nYaw: %5.2f Roll: %5.2f Pitch: %5.2f p: %5.2f q: %5.2f r: %5.2f \nSys: %i Gyro: "
        "%i Accel: %i Mag: %i X_acc: %f Y_acc: %f Z_acc: %f\n ",
         substate.imu.yaw,	substate.imu.roll,	substate.imu.pitch,
         substate.imu.p, 		substate.imu.q,			substate.imu.r,
         substate.imu.sys,	substate.imu.gyro,	substate.imu.accel,
         substate.imu.mag,	substate.imu.x_acc,	substate.imu.y_acc,
         substate.imu.z_acc);
*/

      // Write a newline
//      output << std::endl;

      // Write to general log file
      genlog << output.str();

      // Write to output
      std::cout << output.str();


      /* Generate an output */

      // Yaw controller info
      gettimeofday(&now, NULL);
      double timestamp = (now.tv_sec - mainstart.tv_sec)*1000 +
                         (now.tv_usec - mainstart.tv_usec)/1000;
      double yaw = substate.imu.yaw;
      double setpoint = yaw_pid.setpoint;

      // Write to file
      yawlog << timestamp << "," << yaw << "," << setpoint << ",";
      yawlog << portmotorspeed << "," << starmotorspeed << std::endl;

    }
		auv_msleep(1000/LOG_RATE);
	}

	// Close log files
	genlog.close();
	yawlog.close();

	pthread_exit(NULL);
}//*/
