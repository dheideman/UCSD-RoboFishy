/******************************************************************************
 * FeatureTracker.h
 * 
 * Try to track linear position using camera...  This can't end well...
 * 
 * Person on the Internet:
 * http://stackoverflow.com/questions/31835114/feature-detection-with-patent-free-descriptors
 *
 * FLANN: 
 * http://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html?highlight=flannbasedmatcher#flannbasedmatcher
 ******************************************************************************/

#include <iostream>
#include <opencv2/opencv.hpp>

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

// Core
#include "../../Modules/Core/Core.h"

// Camera
#include "../../Modules/Camera/Camera.h"

// Odometry
#include "../../Modules/Odometry/Odometry.h"


#define DEBUG

#define OUTPUT_VIDEO_NAME "DemoVideo.avi"
//#define FRAME_WIDTH       640
//#define FRAME_HEIGHT      480

#define MATCH_CIRCLE_R    5      // Match circle radius, px
