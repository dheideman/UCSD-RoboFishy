/*****************************************************************************
 * Camera.h
 *
 * Module: Camera
 * 
 * Pull images from camera as soon as they are taken and store them safely
 * 
 ******************************************************************************/
 
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

// Exposure Values
#define BRIGHT_EXPOSURE 100
#define DARK_EXPOSURE   5
#define ISO_VALUE       4

// Image Size
//#define FRAME_WIDTH     3280  // 8 megapixels
//#define FRAME_HEIGHT    2464  // 8 megapixels
#define FRAME_WIDTH       1280  // 720 HD
#define FRAME_HEIGHT      720   // 720 HD

// Laser Pin
#define LASERPIN      4


/////////////////////////
// Function Prototypes //
/////////////////////////

// Threads
void *takePictures(void*);


//////////////////////
// Global Variables //
//////////////////////

// Image Capture Object
VideoCapture cap;

// V4L2 Global Device Object
V4L2Control picamctrl;

// Image Storage
sub_images_t subimages;

