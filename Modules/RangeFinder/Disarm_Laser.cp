/************************ *****************************************************
 * Disarm_Laser.cpp
 *
 * Module: Camera/Mapper
 * Arm the Laser for safe operation when the sub is not oriented vertically
 ******************************************************************************/

#include "../../Electronics/Mapper.h"
#include "../Core/Core.h"

#define PITCH_SATURATION 30		// Kill laser at 30 degree pitch
#define ROLL_SATURATION 30		// Kill laser at 30 degree roll

//////////////////////
// Global Variables // 
//////////////////////

// Declare Global State Variables
// sub_state_t substate;
// bno055_t bno055;

 /*******************************************************************************
 * void *takePictures(void*)
 * 
 * Camera-handling thread: continuously save images as soon as they come in
 ******************************************************************************/
void disarmLaser(void* arg)
{
  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
  	// If we exceed pitch or roll setpoints disarm the laser as to not blind anyone
    if( bno055.pitch > PITCH_SATURATION || bno055.roll > ROLL_SATURATION ){
      
    	// Disarm the laser
    	substate.laserarmed = DISARMED;
    }

    usleep(2500);
  }
  pthread_exit(NULL);
}

