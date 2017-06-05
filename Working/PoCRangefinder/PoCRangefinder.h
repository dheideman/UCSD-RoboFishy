/*****************************************************************************
 * PoCRangefinder.h
 *
 * Take images at varying distances from target to calibrate laser stick
 * 
 ******************************************************************************/

// OpenCV
#include <cv.hpp>
//#include <highgui.h>
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

// V4L2Control
#include <linux/videodev2.h>
#include "../../Modules/V4L2Control/V4L2Control.h"

// RangeFinder
#include "../../Modules/RangeFinder/RangeFinder.h"

// Camera
#include "../../Modules/Camera/Camera.h"


///////////////
// Constants //
///////////////

// Window Name
#define SOURCE_WINDOW     "Bright Image"


/////////////////////////
// Function Prototypes //
/////////////////////////

// Callback Functions
void whiteBalanceCallback(int, void*);
void mouseCallback(int event, int x, int y, int flags, void* userdata);
void exposureCallback(int, void*);

// Threads
void *takePictures(void*);
