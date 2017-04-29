/******************************************************************************
 * ExposureLaser.cpp
 *
 * Play with camera parameters, find laser dot
 *
 ******************************************************************************/

#include <cv.hpp>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <raspicam/raspicam_cv.h>

using namespace cv;
using namespace std;

// Global Variables
int n = 100;
string source_window = "Original Image";
string detect_window = "Detected Laser Pixels";
Mat frame, hsv_frame;
raspicam::RaspiCam_Cv cap;
int exposure = 1;
int gain = 1; 
int brightness, contrast;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == EVENT_LBUTTONDOWN )
  {
    Vec3b intensity = hsv_frame.at<Vec3b>(y, x);
    int hue = intensity.val[0];
    int sat = intensity.val[1];
    int val = intensity.val[2];
    cout << "H: " << hue << ",\tS:" << sat << ",\tV:" << val << endl;
    
    Vec3b bgr = frame.at<Vec3b>(y, x);
    int b = bgr.val[0];
    int g = bgr.val[1];
    int r = bgr.val[2];
    cout << "B: " << b << ",\tG:" << g << ",\tR:" << r << endl;
  }
}

void exposureCallback(int, void* )
{
  cap.set(CV_CAP_PROP_EXPOSURE,(float) exposure/10);
}

void gainCallback(int, void* )
{
  cap.set(CV_CAP_PROP_GAIN, (float) gain);
}

void brightnessCallback(int, void* )
{
  cap.set(CV_CAP_PROP_BRIGHTNESS, (float) brightness);
}

void contrastCallback(int, void* )
{
  cap.set(CV_CAP_PROP_CONTRAST, (float) contrast);
}

int main(int argc, char** argv)
{
  if ( !cap.open() )
  {
    cerr << "Error opening the camera" << endl;
    return -1;
  }

  // Set capture camera properties to maximum (8 MP)
//  cap.set(CV_CAP_PROP_FRAME_WIDTH,3280);
//  cap.set(CV_CAP_PROP_FRAME_HEIGHT,2464);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
//  cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
//  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  
  // Open windows
//  namedWindow( source_window, CV_WINDOW_NORMAL );
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
//  namedWindow( detect_window, CV_WINDOW_AUTOSIZE );

  //set the callback function for any mouse event
//  setMouseCallback(source_window, CallBackFunc, NULL);
  
  // Create trackbars
  createTrackbar( " Exposure:", source_window, &exposure, 100, exposureCallback );
  createTrackbar( " Gain:", source_window, &gain, 50, gainCallback );
  //createTrackbar( " Brightness:", source_window, &brightness, 100, brightnessCallback );
  //createTrackbar( " Contrast:", source_window, &contrast, 100, contrastCallback );

  // initialize stop key variable
  int key = 0;

  // Take a bunch of pictures
  while(key!=27)
  {
    // Image stuff
    cap.grab();
    cap.retrieve ( frame );
    
    // Convert BGR to HSV
    cvtColor(frame, hsv_frame, CV_BGR2HSV);
    
    // Find laser dot
    Mat mask;
    inRange(hsv_frame, Scalar(65, 30, 128), Scalar(89, 200, 240), mask);
    
    // Locate centroid of laser dot
    Moments m = moments(mask, false);
    Point p1(m.m10/m.m00, m.m01/m.m00);
    
    // Draw circle w/ center at centroid on original image
    circle( frame,p1,10.0,Scalar( 0, 0, 255 ));
    
    // Write coordinates of center of laser to terminal
    cout << "(" << p1.x << ", " << p1.y << ")" << endl;
    // Show images on screen
    imshow(source_window,frame);
//    imshow(detect_window,mask);

    key = waitKey(1);
  }
}
