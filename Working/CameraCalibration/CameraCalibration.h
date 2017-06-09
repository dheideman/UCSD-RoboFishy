/*****************************************************************************
 * CameraCalibration.h
 *
 * Set white balance for given lighting, write parameters to file
 * https://web.stanford.edu/~sujason/ColorBalancing/robustawb.html
 ******************************************************************************/

// OpenCV
#include <cv.hpp>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"

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

// Core Module
#include "../../Modules/Core/Core.h"

// V4L2 Module
#include <linux/videodev2.h>
#include "../../Modules/V4L2Control/V4L2Control.h"

// Camera Module
#include "../../Modules/Camera/Camera.h"

/////////////////////////
// Function Prototypes //
/////////////////////////

// Callback Functions
void mouseCallback(int event, int x, int y, int flags, void* userdata);


