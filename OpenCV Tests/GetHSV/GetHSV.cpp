/******************************************************************************
 * GetHSV.cpp
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
string source_window = "Get HSV Values";
Mat hsv_frame;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
  //cout << "Event Detected!" << endl;
  if  ( event == EVENT_LBUTTONDOWN )
  {
    Vec3b intensity = hsv_frame.at<Vec3b>(y, x);
    int hue = intensity.val[0];
    int sat = intensity.val[1];
    int val = intensity.val[2];
    cout << "H: " << hue << ",\tS:" << sat << ",\tV:" << val << endl;
  }
}

int main(int argc, char** argv)
{
  if(argc < 2)
  {
    cout << "Not enough arguments.  Usage: ./GetHSV <video source>" << endl;
    return -1;
  }
  
  // open the video file for reading
  int videosource;
  sscanf(argv[1], "%d", &videosource);
  
  VideoCapture cap = VideoCapture(videosource);
  
  if ( !cap.isOpened() )  // if not success, exit program
  {
    cout << "Cannot open the video file" << endl;
    return -1;
  }
  
  // Set capture camera properties to maximum (8 MP)
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,3280);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,2464);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);

  cout << "Click anywhere on the image to get the HSV values at that point." << endl;
  
  /// Create a window and a trackbar
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  
  //set the callback function for any mouse event
  setMouseCallback(source_window, CallBackFunc, NULL);
  
  // Initialize image matrix
  Mat frame;

  int key = 0;
  while(key != 27)
  { 
    bool bSuccess = cap.read(frame); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read the frame from video file" << endl;
      break;
    }
    // Convert BGR to HSV
    cvtColor(frame, hsv_frame, CV_BGR2HSV);

    imshow( source_window, frame );
    key = waitKey(10);
  }
  // Display results
  cout << "Thank you, come again!" << endl;
  
}
