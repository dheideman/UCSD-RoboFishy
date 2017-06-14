/******************************************************************************
 * User Interface Thread
 * void* userInterface(void* arg)
 *
 * Interfaces with the user, asks for input
 *****************************************************************************/
 void* userInterface(void* arg)
 {
  // Declare local constant variables
  float _kp = 0.0003, _ki = 0, _kd = 0;

  // Wait a until everything is initialized before starting
  while(substate.mode == INITIALIZING)
  {
    // Waiting...
    auv_msleep(100);
  }

  // Prompt user for values continuously until the program exits
  while(substate.mode != STOPPED)
  {
    // Print Kp, Ki, Kd for reference
    std::cout << "Current Values:" << std::endl;
    printf("Kp:\t%f\tKi:\t%f\tKd:\t%f\n",_kp,_ki,_kd);
    std::cout << std::endl;

    // Prompt for kp
    std::cout << "Kp: ";
    std::cin >> _kp;

    // Prompt for ki
    std::cout << "Ki: ";
    std::cin >> _ki;

    // Prompt for kd
    std::cout << "Kd: ";
    std::cin >> _kd;

    // Give a newline
    std::cout << std::endl;

    // Reset gain values to input values
    yaw_pid.kp = _kp;
    yaw_pid.ki = _ki;
    yaw_pid.kd = _kd;

    // Clear errors
    yaw_pid.perr = 0;
    yaw_pid.ierr = 0;
    yaw_pid.derr = 0;
		depth_pid.perr = 0;
		depth_pid.ierr = 0;
		depth_pid.derr = 0;

    // Start RUNNING again
    substate.mode = RUNNING;

    // Restart timer!
	  //gettimeofday(&start, NULL);

		// Wait for mode to pause again
		while (substate.mode == RUNNING) {
			auv_msleep(100);
		}

    auv_msleep(1000);
  }

  // Exit thread
  pthread_exit(NULL);
 }
