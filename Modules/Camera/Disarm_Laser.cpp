/************************ *****************************************************
 * Disarm_Laser.cpp
 *
 * Module: Camera/Mapper
 * Arm the Laser for safe operation when the sub is not oriented vertically
 ******************************************************************************/

//#include "RangeFinder.h"
//#include "../../Electronics/Mapper.h"
#include "Camera.h"
#include "../Core/Core.h"

#define PITCH_SATURATION 30		// Kill laser at 30 degree pitch
#define ROLL_SATURATION 30		// Kill laser at 30 degree roll

//////////////////////
// Global Variables // 
//////////////////////

// Declare Global State Variables
// sub_state_t substate;
// imu_t bno055;

/*******************************************************************************
 * void *disarmLaser(void*)
 * 
 * Camera-handling thread: continuously save images as soon as they come in
 ******************************************************************************/
void *disarmLaser(void* arg)
{
  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
  	// If we exceed pitch or roll setpoints disarm the laser as to not blind anyone
    if( substate.imu.pitch > PITCH_SATURATION || substate.imu.roll > ROLL_SATURATION ){
      
    	// Disarm the laser
    	substate.laserarmed = DISARMED;
//      printf("Laser Disarmed!\n");
    }
    else
    {
      // Re-arm the laser
      substate.laserarmed = ARMED;
//      printf("Laser Armed!\n");
    }

    usleep(100000);
  }
  pthread_exit(NULL);
}

