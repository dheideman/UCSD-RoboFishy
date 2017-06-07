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
  float yaw, roll, pitch, p, q, r;
  int sys, gyro, accel, mag;
}bno055_t;

// Submersible overall state type
typedef struct sub_state_t
{
  sub_mode_t    mode;           // Operating mode
  double        range;          // Range to bottom
  double        depth;          // Depth below surface
  cv::Point3f   pose;           // Location + Yaw of sub
  armed_t       laserarmed;     // Whether the laser can be turned on or not
  bno055_t      imuorientation; // Orientation as determined by IMU
} sub_state_t;

// Struct for holding current system state
typedef struct system_state_t
{
  float roll;         // current roll angle (rad)
  float pitch[2];       // current pitch angle (rad) 0: current value, 1: last value
  float yaw[2];       // current yaw angle (rad) 0: current value, 1: last value
  float depth[2];       // depth estimate (m)
  float fdepth[2];      // filtered depth estimate (m)
  float speed;        // speed (m/s)

  float p[2];         // first derivative of roll (rad/s)
  float q[2];         // first derivative of pitch (rad/s)
  float r[2];         // first derivative of yaw (rad/s)
  float ddepth;       // first derivative of depth (m/s)

  int sys;    // system calibrations status (0=uncalibrated, 3=fully calibrated)
  int gyro;   // gyro calibrations status (0=uncalibrated, 3=fully calibrated)
  int accel;    // accelerometer calibrations status (0=uncalibrated, 3=fully calibrated)
  int mag;    // magnetometer calibrations status (0=uncalibrated, 3=fully calibrated)
} system_state_t;

/////////////////////////
// Function Prototypes //
/////////////////////////

// PThread Priority Attributes Initializer
void initializeTAttr();

// PThread Priority Attributes Destroyer
void destroyTAttr();


//////////////////////
// Global Variables //
//////////////////////

// Global State Variable
extern sub_state_t substate;

// Holds the system state structure with current system state
extern system_state_t sstate;

// Thread attributes for different priorities
extern pthread_attr_t tattrlow, tattrmed, tattrhigh;

#endif
