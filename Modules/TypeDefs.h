/*****************************************************************************
 * TypeDefs.h
 *
 * Define global submersible types
 * 
 ******************************************************************************/

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
