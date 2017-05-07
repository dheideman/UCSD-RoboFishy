/******************************************************************************
 * SavePictures.cpp
 * 
 * http://answers.opencv.org/question/34461/how-to-set-camera-resolution-webcam-with-opencv/
 ******************************************************************************/

#include <cv.hpp>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

// Global Variables
int n = 100;
string filebasename = "images/image";
string extension = ".jpg";

int main(int argc, char** argv)
{
  VideoCapture cap = VideoCapture(0); // open the video file for reading
  
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

  cout << "Preparing to save " << n << " pictures." << endl;

  // Initialize timers
  time_t time_begin, time_end;
  time(&time_begin);
  
  

  // Take a bunch of pictures
  for(int i=0; i<n; i++)
  {
    // Initialize image stuff
    Mat frame;
    bool bSuccess = cap.read(frame); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
        cout << "Cannot read the frame from video file" << endl;
        break;
    }
    
    // Write last image to file
    std::ostringstream filenamestream;
    if(i<10)
      filenamestream << filebasename << "0" << i << extension;
    else
      filenamestream << filebasename << i << extension;
    imwrite(filenamestream.str(),frame);
  }

  // Measure stop time
  time(&time_end);
  double secondselapsed = difftime(time_end,time_begin);

  // Display results
  cout << "Time to take and write " << n << " pictures: " << secondselapsed << " seconds" << endl;
  cout << "Time per picture: " << secondselapsed/(float) n << " seconds" << endl;
}
