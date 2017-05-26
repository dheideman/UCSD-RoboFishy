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
#include "../../Modules/TypeDefs.h"

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// V4L2
#include <linux/videodev2.h>
#include "../../Modules/V4L2Control/V4L2Control.h"

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

<<<<<<< HEAD
// Laser Armed enumerated type
=======
// Armed? enumerated type
>>>>>>> f2abf42a11d48ec6e9b2c1fd81a10f7d8a02ebbe
typedef enum armed_t
{
  ARMED,
  DISARMED
} armed_t;

// Submersible overall state type
typedef struct sub_state_t
{
<<<<<<< HEAD
  sub_mode_t mode;
  double    range;
  armed_t   laserarmed;
=======
  sub_mode_t  mode;           // Operating mode
  double      range;          // Range to bottom
  double      depth;          // Depth below surface
//   cv::Mat     pose;           // Location + Yaw of sub
  armed_t     laserarmed;     // Whether the laser can be turned on or not
//   cv::Mat     imuorientation; // Orientation as determined by IMU
>>>>>>> f2abf42a11d48ec6e9b2c1fd81a10f7d8a02ebbe
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
//void *rangeFinder(void*);
