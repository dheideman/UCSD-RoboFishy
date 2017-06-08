/*****************************************************************************
 * Core.h
 *
 * Core Module: Defines global submersible types
 *
 ******************************************************************************/

#ifndef AUVCORE_H
#define AUVCORE_H

//#include <cv.hpp>
#include <opencv2/opencv.hpp>
#include <signal.h>   

///////////////
// Constants //
///////////////

// Math
#define PI    3.141592653589793


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

// Struct for holding BNO055 return values
typedef struct
{
  float yaw, roll, pitch, p, q, r, x_acc, y_acc, z_acc;
  int sys, gyro, accel, mag;
} imu_t;

// Submersible overall state type
typedef struct sub_state_t
{
  sub_mode_t    mode;           // Operating mode
  double        range;          // Range to bottom
  double        depth;          // Depth below surface
  double        fdepth;         // Filtered depth below surface
  cv::Point3f   pose;           // Location + Yaw of sub
  armed_t       laserarmed;     // Whether the laser can be turned on or not
  imu_t      imu; // Orientation as determined by IMU
} sub_state_t;

/////////////////////////
// Function Prototypes //
/////////////////////////

// PThread Priority Attributes Initializer
void initializeTAttr();

// PThread Priority Attributes Destroyer
void destroyTAttr();

// signal catcher
void ctrl_c(int signo);     

//////////////////////
// Global Variables //
//////////////////////

// Global State Variable
extern sub_state_t substate;

// Thread attributes for different priorities
extern pthread_attr_t tattrlow, tattrmed, tattrhigh;

#endif
