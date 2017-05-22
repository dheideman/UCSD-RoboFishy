/******************************************************************************
 * OpticalFlowDemo.cpp
 * 
 * Try to track linear position using camera...  This can't end well...
 * 
 * http://funvision.blogspot.dk/2016/02/opencv-31-tutorial-optical-flow.html
 * 
 * Tracker: 
 * http://docs.opencv.org/3.1.0/dc/d6b/group__video__track.html
 ******************************************************************************/
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <vector>
#include <stdio.h>
#include <iostream>

#define WINDOW_NAME   "Optical Flow Demo"
#define FRAME_WIDTH   1280
#define FRAME_HEIGHT  720
#define CAMERA_FPS    30

using namespace cv;
using namespace std;

//////////
// Main //
//////////
int main(int argc, const char** argv)
{
  // Make sure we have at least 2 input arguments
  if( argc < 2 )
  {
    cout << "Usage: ./OpticalFlow <video feed #>" << endl;
    return -1;
  }
  
  // Open video feed
  VideoCapture cap(atoi(argv[1]));
  
  if( !cap.isOpened() )
  {
    cout << "Could not open camera" << endl;
    return -1;
  }
  
  // Set camera settings
  cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
  cap.set(CV_CAP_PROP_FPS,CAMERA_FPS);
  
  Mat rgbframe;
  Mat greyframe[2];   // 0: new, 1: old
  Mat flow;
  
  // Make sure buffer is clear
//   for(int i=0; i<5; i++) { cap.grab(); }
  
  // Get a starting "previous image"
  cap.read(rgbframe);
  cvtColor(rgbframe, greyframe[1], COLOR_BGR2GRAY);
  
  // Create window
  namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
  
  // Wait one second
  sleep(1);
  
  while( waitKey(1) != 27 )
  { 
    // capture frame from video file
    cap.read(rgbframe);
//     resize(rgbframe, rgbframe, Size(640, 480));
    
    // just make current frame gray
    cvtColor(rgbframe, greyframe[0], COLOR_BGR2GRAY); 
    
    // calculate optical flow 
    calcOpticalFlowFarneback( greyframe[1], greyframe[0], flow,
                              0.4, 1, 12, 2, 8, 1.2, 0);
                       
    // Draw flow vectors
    for (int y = 0; y < rgbframe.rows; y += 5)
    {
      for (int x = 0; x < rgbframe.cols; x += 5)
      {
        // get the flow from y, x position * 10 for better visibility
        const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
        
        // draw line at flow direction
        line(rgbframe, Point(x, y),
             Point( cvRound(x + flowatxy.x),cvRound(y + flowatxy.y) ),
             Scalar(255,0,0));
             
        // draw initial point
        circle(rgbframe, Point(x, y), 1, Scalar(0, 0, 0), -1);
      }
    }
    
    // draw the results
    imshow(WINDOW_NAME, rgbframe);
    
    // save current grey frame to "old" grey frame
    cv::swap(greyframe[0],greyframe[1]);
    
  } // end while()
}