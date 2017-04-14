/******************************************************************************
 * FindLaser.cpp
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
string source_window = "Display Pixels In Range";
Mat hsv_frame;
int morph_elem = 0;
int morph_size = 2;

int main(int argc, char** argv)
{
  if(argc < 2)
  {
    cout << "Not enough arguments.  Usage: ./FindLaser <video source>" << endl;
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
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

  cout << "Showing laser position estimate." << endl;
  
  /// Create a window and a trackbar
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
    
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
    //cvtColor(frame, hsv_frame, CV_BGR2HSV);
    
    //Mat1b mask, mask1, mask2;
    //inRange(hsv_frame, Scalar(0, 40, 200), Scalar(10, 255, 255), mask1);
    //inRange(hsv_frame, Scalar(170, 40, 200), Scalar(180, 255, 255), mask2);
    //inRange(hsv_frame, Scalar(75, 140, 200), Scalar(95, 255, 255), mask1);
    //inRange(hsv_frame, Scalar(0, 0, 255), Scalar(0, 0, 255), mask2);
    
    //mask = mask1 | mask2;
    //mask = mask2;
    
    // Split image into channels
    vector<Mat> channels;
    split(frame,channels);
    
    // Select red channel
    int ch = 1;
    
    // Subtract mean of other two channels from selected one
    Mat mask = channels[ch];
    for(int i=0; i<3; i++)
    {
      //if(i!=ch)
      //{
        mask -= channels[i]/3;
      //}
    }
    
    //Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

    /// Apply the specified morphology operation
    //morphologyEx( mask, frame, 2, element );
    
    imshow( source_window, mask );
    key = waitKey(10);
  }
  // Display results
  cout << "Thank you, come again!" << endl;
  
}
