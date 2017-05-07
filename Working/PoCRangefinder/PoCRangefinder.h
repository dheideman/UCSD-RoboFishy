/*****************************************************************************
 * PoCRangefinder.h
 *
 * Take images at varying distances from target to calibrate laser stick
 * 
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

// Submersible overall state type
typedef struct sub_state_t
{
  sub_mode_t mode;
} sub_state_t;

/////////////////////////
// Function Prototypes //
/////////////////////////

// Callback Functions
void whiteBalanceCallback(int, void*);
void mouseCallback(int event, int x, int y, int flags, void* userdata);

// Threads
void *takePictures(void*);
