/*****************************************************************************
 * CameraCalibration.h
 *
 * Set white balance for given lighting, write parameters to file
 * https://web.stanford.edu/~sujason/ColorBalancing/robustawb.html
 ******************************************************************************/

#include <cv.hpp>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <ostream>
#include <string>

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include "V4L2Control.h"

/////////////////////////
// Function Prototypes //
/////////////////////////

// Callback Functions
void whiteBalanceCallback(int, void*);
void mouseCallback(int event, int x, int y, int flags, void* userdata);

// Threads
void *takePictures(void*);
