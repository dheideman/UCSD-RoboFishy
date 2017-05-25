/*****************************************************************************
 * PoCRangefinder.h
 *
 * Take images at varying distances from target to calibrate laser stick
 * 
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

// Global typedefs
#include "../../../Modules/TypeDefs.h"

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// V4L2
#include <linux/videodev2.h>
#include "../../../Modules/V4L2Control/V4L2Control.h"

//////////////////////
// Type definitions //
//////////////////////

// Submersible operating mode enumerated type
typedef enum sub_mode_t
{
  INITIALIZING,
  RUNNING,
  PAUSED,
  CONFUSED,
  PANICKING,
  STOPPED
} sub_mode_t;

// Armed? enumerated type
typedef enum armed_t
{
  ARMED,
  DISARMED
} armed_t;

// Submersible overall state type
typedef struct sub_state_t
{
  sub_mode_t  mode;           // Operating mode
  double      range;          // Range to bottom
  double      depth;          // Depth below surface
//   cv::Mat     pose;           // Location + Yaw of sub
  armed_t     laserarmed;     // Whether the laser can be turned on or not
//   cv::Mat     imuorientation; // Orientation as determined by IMU
} sub_state_t;

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
