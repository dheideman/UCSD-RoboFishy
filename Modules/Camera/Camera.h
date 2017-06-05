/*****************************************************************************
 * Camera.h
 *
 * Module: Camera
 * 
 * Pull images from camera as soon as they are taken and store them safely
 * 
 ******************************************************************************/

#ifndef CAMERA_H
#define CAMERA_H

// OpenCV
//#include <cv.hpp>
#include <opencv2/opencv.hpp>
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

// V4L2
#include <linux/videodev2.h>
#include "../../Modules/V4L2Control/V4L2Control.h"

// Core
#include "../../Modules/Core/Core.h"


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


//////////////////////
// Type Definitions //
//////////////////////

// Bright and Dark Frame storage struct
typedef struct sub_images_t
{
  cv::Mat brightframe;  // The brighter image (used for mapping)
  cv::Mat darkframe;    // The darker image (used for range finding)
  
  pthread_mutex_t brightframelock;   // Mutex lock for brightframe
  pthread_mutex_t darkframelock;     // Mutex lock for darkframe
} sub_images_t;


/////////////////////////
// Function Prototypes //
/////////////////////////

// Thread
void *takePictures(void*);

// Mutex Lock Initializer
int initializeSubImagesLock(sub_images_t *_subimages);


//////////////////////
// Global Variables //
//////////////////////

// Image Capture Object
extern cv::VideoCapture cap;

// V4L2Control Object
extern V4L2Control picamctrl;

// Image Storage
extern sub_images_t subimages;

#endif
