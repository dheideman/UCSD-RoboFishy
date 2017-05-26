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
#include "opencv2/calib3d.hpp"
#include <vector>
#include <stdio.h>
#include <iostream>

#define WINDOW_NAME   "Optical Flow Demo"
#define FRAME_WIDTH   640
#define FRAME_HEIGHT  480
#define CAMERA_FPS    15

#define MAX_CORNERS   1000
#define MAX_LEVELS    10

#define OUTPUT_VIDEO_NAME "DemoVideo.avi"

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
  
  // Set video capture settings:
/*
  // Get Codec Type- Int form
  int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
  
  // Acquire input size 
  Size framesize = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH),
                        (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  
  // Get FPS
  float fps = cap.get(CV_CAP_PROP_FPS);
  
  // Initialize video writer
  VideoWriter outputvideo;
  outputvideo.open(OUTPUT_VIDEO_NAME, ex, fps, framesize, true);
*/
  
  // Initialize optical flow variables
  Mat rgbframe;
  Mat greyframe[2];   // 0: new, 1: old
  vector<Mat> greypyr0, greypyr1;
  vector<Point2f> corners[2];
  Mat flow;
  
  TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
  Size subwinsize(10,10), winsize(31,31);
  
  // Make sure buffer is clear
//   for(int i=0; i<5; i++) { cap.grab(); }
  
  // Wait one second
  sleep(1);
  
  // Get a starting "previous image"
  cap.read(rgbframe);
  cvtColor(rgbframe, greyframe[1], COLOR_BGR2GRAY);
  
  // Create window
  namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
  
  while( waitKey(1) != 27 )
  { 
    // capture frame from video file
    cap.read(rgbframe);
//     resize(rgbframe, rgbframe, Size(640, 480));
    
    // just make current frame gray
    cvtColor(rgbframe, greyframe[0], COLOR_BGR2GRAY); 
    
    // Detect features
    goodFeaturesToTrack(greyframe[1], corners[1], MAX_CORNERS, 0.01, 10);

    if( corners[1].size() != 0 )
    {
      // Calculate the refined corner locations
      cornerSubPix(greyframe[1], corners[1], subwinsize, Size(-1,-1), termcrit);
      
      // Generate image pyramids
      buildOpticalFlowPyramid(greyframe[0], greypyr0, winsize, MAX_LEVELS);
      buildOpticalFlowPyramid(greyframe[1], greypyr1, winsize, MAX_LEVELS);
      
      // calculate optical flow 
      vector<uchar> status;
      vector<float> err;
//       calcOpticalFlowPyrLK( greyframe[1], greyframe[0], corners[1], corners[0],
//                             status, err, winsize, 3, termcrit, 0, 0.001 );
      calcOpticalFlowPyrLK( greypyr1, greypyr0, corners[1], corners[0],
                            status, err, winsize, 3, termcrit, 0, 0.001 );
//       calcOpticalFlowFarneback( greyframe[1], greyframe[0], flow,
//                                 0.4, 10, 120, 2, 8, 1.2, 0);
//                        
//       // Draw flow vectors
//       for (int y = 0; y < rgbframe.rows; y += 5)
//       {
//         for (int x = 0; x < rgbframe.cols; x += 5)
//         {
//           // get the flow from y, x position * 10 for better visibility
//           const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
//           
//           // draw line at flow direction
//           line(rgbframe, Point(x, y),
//                Point( cvRound(x + flowatxy.x),cvRound(y + flowatxy.y) ),
//                Scalar(255,0,0));
//              
//           // draw initial point
//           circle(rgbframe, Point(x, y), 1, Scalar(0, 0, 0), -1);
//         }
//       }
    
      for(int i=0; i<corners[1].size(); i++)
      {
        if(status[i]==1)
        {
          line( rgbframe, corners[1][i], corners[0][i], Scalar(255,0,0) );
        }
      }
      
      // Find perspective transformation between two planes.
      Mat H = findHomography(corners[1], corners[0], CV_RANSAC);
      
      // Create point arrays for image corners
      vector<Point2f> boxcorners(4);
      
      // Create array of corner points of object image
      boxcorners[0] = cvPoint(0, 0);
      boxcorners[1] = cvPoint(FRAME_WIDTH, 0);
      boxcorners[2] = cvPoint(FRAME_WIDTH, FRAME_HEIGHT);
      boxcorners[3] = cvPoint(0, FRAME_HEIGHT);
    
      // Transform corner points by H to get corresponding points on scene image
      perspectiveTransform(boxcorners, boxcorners, H);
      
      // Draw box around detected object in scene image
      line(rgbframe, boxcorners[0], boxcorners[1], Scalar(0, 0, 255), 4);
      line(rgbframe, boxcorners[1], boxcorners[2], Scalar(0, 0, 255), 4);
      line(rgbframe, boxcorners[2], boxcorners[3], Scalar(0, 0, 255), 4);
      line(rgbframe, boxcorners[3], boxcorners[0], Scalar(0, 0, 255), 4); 
    }
    
    // draw the results
    imshow(WINDOW_NAME, rgbframe);
    
    // Write frame to video
//     outputvideo.write(rgbframe);
        
    // save current grey frame to "old" grey frame
    cv::swap(greyframe[0],greyframe[1]);
    
  } // end while()
}