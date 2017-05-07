/******************************************************************************
 * FindDiff.cpp
 * 
 * Take one picture without and one with a laser to find the laser point.
 * 
 * http://opencv-srf.blogspot.com/2011/11/mouse-events.html
 ******************************************************************************/

#include <cv.hpp>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

// Global Variables
string source_window = "Laser Dot Difference";
VideoCapture cap;


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
  // Initialize image matrices
  Mat src1, src2, diff, frame;
  
  if  ( event == EVENT_LBUTTONDOWN )
  {
    bool bSuccess;
    
    // Read first image (no laser)
    bSuccess = cap.read(src1);
    bSuccess = cap.read(src2);
    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read the frame from video file" << endl;
      return;
    }
    //imshow( source_window, src1 );
    
    // Wait 1 second to let user turn on laser
    //waitKey(1000);
    
    // Read second image (one with laser dot)
    //bSuccess = cap.read(src2);
    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read the frame from video file" << endl;
      return;
    }
    absdiff(src1, src2, diff);
    imshow( source_window, diff );
  }
}
int main(int argc, char** argv)
{
  if(argc < 2)
  {
    cout << "Not enough arguments.  Usage: ./FindDiff <video source>" << endl;
    return -1;
  }
  
  // open the video file for reading
  int videosource;
  sscanf(argv[1], "%d", &videosource);
  
  cap.open(videosource);
  
  if ( !cap.isOpened() )  // if not success, exit program
  {
    cout << "Cannot open the video file" << endl;
    return -1;
  }
  
  // Set capture camera properties to maximum (8 MP)
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

  cout << "Preparing to take pictures." << endl;
  
  /// Create a window
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  
  //set the callback function for any mouse event
  setMouseCallback(source_window, CallBackFunc, NULL);
  
  // Put something in the window.
  Mat frame;
  if (!cap.read(frame)) //if not success, break loop
  {
    cout << "Cannot read the frame from video file" << endl;
    return -1;
  }
  imshow( source_window, frame );
  
  // Wait for escape key to be pressed.
  int key = 0;
  while(key != 27)
  { 
    key = waitKey(100);
  }
  
  // Display results
  cout << "Okay, bye!" << endl;
  
}
