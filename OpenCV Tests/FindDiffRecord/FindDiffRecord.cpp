/******************************************************************************
 * FindDiffRecord.cpp
 * 
 * Take one picture without and one with a laser to find the laser point.
 * Record to a file
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
string filename = "test.avi";


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
  
  // Create remapping matrices
  int width  = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
  
  // Initialize file writing stuff
  VideoWriter outputVideo;
  Size S = Size( width, height);
  int openattempts = 0;
  while(!outputVideo.isOpened() && openattempts<10)
  {
    outputVideo.open(filename, ex, cap.get(CV_CAP_PROP_FPS), S, true);
    openattempts++;
  }
  if(!outputVideo.isOpened())
  {
    cout << "Failed to open output video file, " << filename << endl;
    return -1;
  }

  //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

  double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
  
  // Wait for escape key to be pressed.
  int key = 0;
  while(key != 27)
  { 
    Mat src1, src2, diff;
    cout << "Picture!" << endl;
    cap.read(src1);
    cap.read(src2);
    absdiff(src1, src2, diff);
    imshow( source_window, diff );
    
    outputVideo.write(diff);
    key = waitKey(1);
  }
  
  // Display results
  cout << "Okay, bye!" << endl;
  
}
