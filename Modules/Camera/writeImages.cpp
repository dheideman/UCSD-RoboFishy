/************************ *****************************************************
 * writeImages.cpp
 *
 * Module: Camera/Mapper
 * Arm the Laser for safe operation when the sub is not oriented vertically
 ******************************************************************************/

//#include "RangeFinder.h"
//#include "../../Electronics/Mapper.h"
#include "Camera.h"
#include "../Core/Core.h"

#define CSV_FILENAME    "calibration_data.csv"
#define IMAGE_PREFIX    "high_resolution/image"
#define IMAGE_EXTENSION ".jpg"

/*******************************************************************************
 * void *writeImages(void*)
 * 
 * Camera-handling thread: continuously save images as soon as they come in
 ******************************************************************************/
void *writeImages(void*)
{
  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
    // Open data output file:
    ofstream datafile;
    datafile.open (CSV_FILENAME);

  	// If we exceed pitch or roll setpoints disarm the laser as to not blind anyone
    if( subimages.imstate == BRIGHTFRAME )
    {
      // Create filename
      stringstream filename;
      filename << IMAGE_PREFIX;
      if(i<10) filename << "0";   // add in a zero to 1-digit numbers
      filename << i << IMAGE_EXTENSION;

      // Write image to file
      imwrite(filename.str(), frame);
    }
    // close csv file
    datafile.close();
    
    auv_usleep(100000);
  }
  pthread_exit(NULL);
}

