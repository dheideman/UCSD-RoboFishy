/******************************************************************************
 * SavePictures.cpp
 * 
 * http://answers.opencv.org/question/34461/how-to-set-camera-resolution-webcam-with-opencv/
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
//string filebasename = "/mnt/usb/image";
//string extension = ".jpg";
string source_window = "Exposure Tests";

int main(int argc, char** argv)
{
  //VideoCapture cap = VideoCapture(0); // open the video file for reading
  raspicam::RaspiCam_Cv cap;

//  if ( !cap.isOpened() )  // if not success, exit program
//  {
//    cout << "Cannot open the video file" << endl;
//    return -1;
//  }
  
  if ( !cap.open() )
  {
    cerr << "Error opening the camera" << endl;
    return -1;
  }

  // Set capture camera properties to maximum (8 MP)
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,3280);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,2464);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
  
  // Open window
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  
  // Initialize frame
  Mat frame;

  // Take a bunch of pictures
  for(int i=0; i<n; i++)
  {
//    cout << "loop" << endl;
    // Set exposure length
    cap.set(CV_CAP_PROP_EXPOSURE, i+1);
    
    cout << cap.get(CV_CAP_PROP_EXPOSURE) << endl;

    // Initialize image stuff
//    bool bSuccess = cap.read(frame); // read a new frame from video
//    
//    if (!bSuccess) //if not success, break loop
//    {
//        cout << "Cannot read the frame from video file" << endl;
//        break;
//    }
    cap.grab();
    cap.retrieve ( frame );

    imshow(source_window,frame);
    waitKey(100);
  }
}
