/*****************************************************************************
 * RangeFinder.h
 *
 * Module: RangeFinder
 * 
 * Take images and identify laser dot to determine range
 * 
 ******************************************************************************/

#ifndef RANGEFINDER_H
#define RANGEFINDER_H

// OpenCV
#include <cv.hpp>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

// WiringPi
#include <wiringPi.h>

// Streams
#include <iostream>
#include <fstream>
#include <sstream>
#include <ostream>
#include <string>

// Global typedefs
#include "../../Modules/TypeDefs.h"

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// V4L2
#include <linux/videodev2.h>
#include "../../Modules/V4L2Control/V4L2Control.h"


///////////////
// Constants //
///////////////

// Rangefinder Constants
#define RANGE_K0          -567.7
#define RANGE_K1          0.09943
#define RANGE_K2          -330.0


/////////////////////////
// Function Prototypes //
/////////////////////////

// Threads
void *rangeFinder(void*);


//////////////////////
// Type Definitions //
//////////////////////
typedef struct local_images_t
{
  cv::Mat brightframe;   // local version of brightframe
  cv::Mat darkframe;     // local version of darkframe 
} local_images_t;


//////////////////////
// Global Variables //
//////////////////////

// For writing image to screen
local_images_t localimages;
Mat darkframe, brightframe;
Mat hsvframe;
Point p1;

#endif