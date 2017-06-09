/************************ *****************************************************
 * writeImage.cpp
 *
 * Module: Camera/Mapper
 * Save the most recent bright frame every time it is read
 ******************************************************************************/

//#include "RangeFinder.h"
//#include "../../Electronics/Mapper.h"
#include "Camera.h"
#include "../Core/Core.h"

// Paramaters
#define CSV_FILENAME    "calibration_data.csv"
#define IMAGE_PREFIX    "images/image"
#define IMAGE_EXTENSION ".jpg"

//////////////////////
// Global Variables // 
//////////////////////
/*******************************************************************************
 * void *disarmLaser(void*)
 * 
 * Camera-handling thread: continuously save images as soon as they come in
 ******************************************************************************/
void *writeImage(void* arg)
{
  // Open data output file:
  ofstream datafile;
  datafile.open (CSV_FILENAME);
  // Write header to csv file
  datafile << "n, distance, xpixel, ypixel\n";

  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
  	// If we exceed pitch or roll setpoints disarm the laser as to not blind anyone
    if( subimage.imstate == BRIGHTFRAME )
    {
      // Create filename
      stringstream filename;
      filename << IMAGE_PREFIX;
      if(i<10) filename << "0";   // add in a zero to 1-digit numbers
      filename << i << IMAGE_EXTENSION;
        
      // Write image to file
      imwrite(filename.str(), frame);
    }

    usleep(100000);
  }
  // close csv file
  datafile.close();

  pthread_exit(NULL);
}

