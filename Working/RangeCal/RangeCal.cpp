/*****************************************************************************
 * RangeCal.cpp
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
#include <string>

// V4L2 Includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

using namespace cv;
using namespace std;

// Parameters
#define N_CAL_DISTANCES 20
#define CAL_DX          0.3048  // meters between measurements
#define CSV_FILENAME    "calibration_data.csv"
#define IMAGE_PREFIX    "images/image"
#define IMAGE_EXTENSION ".jpg"

// Global Variables
int n;
string source_window = "Original Image";
VideoCapture cap;

// V4L2 Global Variables
int fd;
struct v4l2_control control;

void setExposure(int exposure)
{
  memset(&control, 0, sizeof (control));
  control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  control.value = exposure;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control))
  {
    cout << "Could not change exposure" << endl;
  }
}

void setISO(int iso)
{
  memset(&control, 0, sizeof (control));
  control.id = V4L2_CID_ISO_SENSITIVITY;
  control.value = iso;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control))
  {
    cout << "Could not change ISO: " << errno << endl;
  }
}


//////////
// Main //
//////////
int main(int argc, char** argv)
{ 
  // Open the camera!
  cap.open(0);
  fd = open("/dev/video0",  O_RDWR /* required */ | O_NONBLOCK, 0);

  if ( !cap.isOpened() )
  {
    cerr << "Error opening the camera (OpenCV)" << endl;
    return -1;
  }
  
  // Set camera exposure control to manual
  control.id = V4L2_CID_EXPOSURE_AUTO;
  control.value = V4L2_EXPOSURE_MANUAL;
  
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control))
  {
    cerr << "Could not set exposure control to manual (V4L2)" << endl;
    return -1;
  }
  
  // Set camera awb to fluorescent
  memset(&control, 0, sizeof (control));
  control.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE;
  control.value = V4L2_WHITE_BALANCE_FLUORESCENT;
  
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control))
  {
    cerr << "Could not set white balance to manual (V4L2): "<< errno  << endl;
    return -1;
  }

  // Set camera iso to manual
  memset(&control, 0, sizeof (control));
  control.id = V4L2_CID_ISO_SENSITIVITY_AUTO;
  control.value = 0;//V4L2_CID_ISO_SENSITIVITY_MANUAL;
  
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control))
  {
    cerr << "Could not set ISO control to manual (V4L2): " << errno << endl;
    return -1;
  }
  
  // Initialize exposure, iso values
  setExposure(1);
  setISO(0);
    
  // Set capture camera properties to maximum (8 MP)
//  cap.set(CV_CAP_PROP_FRAME_WIDTH,3280);
//  cap.set(CV_CAP_PROP_FRAME_HEIGHT,2464);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
//  cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
//  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  
  // Open data output file:
  ofstream datafile;
  datafile.open (CSV_FILENAME);
  
  // Write header to csv file
  datafile << "n, distance, xpixel, ypixel\n";

  // Take a bunch of pictures
  for(int i=1; i<=N_CAL_DISTANCES; i++)
  {
    // Calculate distance from which to take the picture (meters)
    float d = i*CAL_DX;
    
    // Write to screen: Prepare to take picture
    cout << "Move to position " << i << ", " << d << " meters from target,\n";
    cout << "Press ENTER to take image.\n";
    
    // Wait for key press
    while(cin.get() != '\n'){}
    
    // Read image into matrix "frame"
    Mat frame;
    cap.grab();
    cap.retrieve( frame );
    
    // Convert from BGR to HSV
    Mat hsv_frame;
    cvtColor(frame, hsv_frame, CV_BGR2HSV);
    
    // Find laser dot
    Mat mask;
    inRange(hsv_frame, Scalar(0, 0, 40), Scalar(180, 255, 255), mask);
    
    // Locate centroid of laser dot
    Moments m = moments(mask, false);
    Point p1(m.m10/m.m00, m.m01/m.m00);
    
    // Draw circle w/ center at centroid on original image
    circle( frame,p1,10.0,Scalar( 0, 0, 255 ));
    
    // Write coordinates of center of laser to file and to terminal
    datafile << i << ", " << d << ", " << p1.x << ", " << p1.y << endl;
    cout << "Laser located at (" << p1.x << ", " << p1.y << ")" << endl;
    
    // Create filename
    stringstream filename;
    filename << IMAGE_PREFIX;
    if(i<10) filename << "0";   // add in a zero to 1-digit numbers
    filename << i << IMAGE_EXTENSION;
        
    // Write image to file
    imwrite(filename.str(), frame);
  } // end for
  
  // close camera
  close(fd);
  
  // close csv file
  datafile.close();
}// end main
