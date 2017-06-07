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
//#include <cv.hpp>
#include <opencv2/opencv.hpp>
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

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// Core
#include "../../Modules/Core/Core.h"

// V4L2
// #include <linux/videodev2.h>
// #include "../../Modules/V4L2Control/V4L2Control.h"

// Camera
#include "../../Modules/Camera/Camera.h"


///////////////
// Constants //
///////////////

// Rangefinder Constants
#define RANGE_K0          -567.7
#define RANGE_K1          0.09943
#define RANGE_K2          -330.0


//////////////////////
// Type Definitions //
//////////////////////

// Local versions of the images for calculation purposes
typedef struct local_images_t
{
  cv::Mat brightframe;   // local version of brightframe
  cv::Mat darkframe;     // local version of darkframe 
} local_images_t;


/////////////////////////
// Function Prototypes //
/////////////////////////

// Threads
void *rangeFinder(void*);
void *disarmLaser(void*);

// Mutex Lock Initializer
  // Coming soon


//////////////////////
// Global Variables //
//////////////////////

// For writing image to screen
extern local_images_t localimages;
extern cv::Mat hsvframe;
extern cv::Point p1;

#endif