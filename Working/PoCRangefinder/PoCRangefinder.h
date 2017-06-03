/*****************************************************************************
 * PoCRangefinder.h
 *
 * Take images at varying distances from target to calibrate laser stick
 * 
 ******************************************************************************/

// OpenCV
#include <cv.hpp>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
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

/////////////////////////
// Function Prototypes //
/////////////////////////

// Callback Functions
void whiteBalanceCallback(int, void*);
void mouseCallback(int event, int x, int y, int flags, void* userdata);
void bcsCallback(int, void*);
void exposureCallback(int, void*);

// Threads
void *takePictures(void*);
void *rangeFinder(void*);

//////////////////////
// Type Definitions //
//////////////////////
typedef struct local_images_t
{
  cv::Mat brightframe;   // local version of brightframe
  cv::Mat darkframe;     // local version of darkframe 
} local_images_t;
