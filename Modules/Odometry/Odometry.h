/******************************************************************************
 * Odometry.h
 * 
 * Track 2D position using down-looking camera.
 * 
 ******************************************************************************/

#ifndef ODOMETRY_H
#define ODOMETRY_H

// OpenCV
#include <opencv2/opencv.hpp>

// Streams
#include <iostream>

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// Timer
#include <sys/time.h>

// Core
#include "../../Modules/Core/Core.h"

// Camera
#include "../../Modules/Camera/Camera.h"


///////////////
// Constants //
///////////////

// Odometry Match Constants
#define MIN_MINDIST       10    // pixels
#define MAX_MATCHES       500

// Odometry Thread Rate
#define ODOM_RATE         1     // Hz

//////////////////////
// Type Definitions //
//////////////////////

// Typedef of struct to store new/old images
typedef struct odom_data_t
{
  cv::Mat                   newimg;       // The newer of the two images stored
  cv::Mat                   oldimg;       // The older of the two images stored
  
  pthread_mutex_t           newimglock;   // Mutex lock for newimg
  pthread_mutex_t           oldimglock;   // Mutex lock for oldimg
  
  std::vector<cv::Point2f>  newpts;       // The matched points in the new image
  std::vector<cv::Point2f>  oldpts;       // The matched points in the new image
  
  cv::Mat                   tf;           // Transformation matrix btwn images
} odom_data_t;


/////////////////////////
// Function Prototypes //
/////////////////////////

// Threads
void *visualOdometry(void*);

// Mutex Lock Initializer
int initializeOdomDataLock(odom_data_t *_odomdata);


//////////////////////
// Global Variables //
//////////////////////

// Global odometry image struct
extern odom_data_t odomdata;

#endif
