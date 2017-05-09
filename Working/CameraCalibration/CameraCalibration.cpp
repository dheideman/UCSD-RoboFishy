/*****************************************************************************
 * CameraCalibration.cpp
 *
 * Set white balance for given lighting, write parameters to file
 * https://web.stanford.edu/~sujason/ColorBalancing/robustawb.html
 ******************************************************************************/

#include "CameraCalibration.h"

using namespace cv;
using namespace std;

////////////////
// Parameters //
////////////////

#define BRIGHT_EXPOSURE 100
#define DARK_EXPOSURE   5
#define ISO_VALUE       1

// Image Size
//#define FRAME_WIDTH     3280  // 8 megapixels
//#define FRAME_HEIGHT    2464  // 8 megapixels
#define FRAME_WIDTH       1280  // 720 HD
#define FRAME_HEIGHT      720   // 720 HD


//////////////////////
// Global Variables //
//////////////////////

// Global Variables
int stopprogram = 0;
string sourcewindow = "Current Image";
VideoCapture cap;
int redbalance = 1600;
int bluebalance = 1600;
int exposure = 5;
Mat frame;
Mat bgrframe;
Mat yuvframe;

// V4L2 Global Device Object
V4L2Control picamctrl;


////////////////////////
// Callback Functions //
////////////////////////

/*******************************************************************************
 * void whiteBalanceCallback(int, void*)
 * 
 * Sets white balance values (from 1-7999)
 * Changing position of trackbar calls this function
 ******************************************************************************/
void whiteBalanceCallback(int, void*)
{
  picamctrl.set(V4L2_CID_RED_BALANCE,redbalance);
  picamctrl.set(V4L2_CID_BLUE_BALANCE,bluebalance);
}

/*******************************************************************************
 * void mouseCallback(int event, int x, int y, int flags, void* userdata)
 * 
 * Write HSV and RGB values of point clicked on to terminal
 * Any mouse movement will call this function
 ******************************************************************************/
void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == EVENT_LBUTTONDOWN ) // Only run when left button is pressed
  {
    Vec3b bgr = bgrframe.at<Vec3b>(y, x);
    int b = bgr.val[0];
    int g = bgr.val[1];
    int r = bgr.val[2];
    // print out RGB values (sanity check)
    cout << "B: " << b << ",\tG:" << g << ",\tR:" << r << endl;
  }
}


//////////////
// Threads! //
//////////////

/*******************************************************************************
 * void *takePictures(void*)
 * 
 * Camera-handling thread: continuously save images as soon as they come in
 ******************************************************************************/
void *takePictures(void*)
{
  // Initialize exposure settings
  picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE );
  
  // Grab all 5 images from the frame buffer in order to clear the buffer
  for(int i=0; i<5; i++)
  {
    cap.grab();
  }
  
  // Loop quickly to pick up images as soon as they are taken
  while(!stopprogram)
  {
    // 'Grab' frame from webcam's image buffer
    cap.grab();
    // Retrieve encodes image from grab buffer to 'frame' variable
    cap.retrieve( frame );
  }
  
  // End thread
  pthread_exit(NULL);
}


//////////
// Main //
//////////
int main(int argc, char** argv)
{
  // Open the camera!
  cap.open(0); // opens first video device
  picamctrl.open("/dev/video0");

  // check to make sure device properly opened
  if ( !cap.isOpened() )
  {
    cerr << "Error opening the camera (OpenCV)" << endl;
    return -1;
  }
  
  // Set framerate (OpenCV capture property)
  cap.set(CV_CAP_PROP_FPS,2);
    
  // Set camera exposure control to manual (driver property)
  picamctrl.set(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  
  // Set camera autowhitebalance to manual
  picamctrl.set(V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,V4L2_WHITE_BALANCE_MANUAL);

  // Set camera iso to manual
  picamctrl.set(V4L2_CID_ISO_SENSITIVITY_AUTO, 0);
  
  // Initialize exposure, iso values
  picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE);
  picamctrl.set(V4L2_CID_ISO_SENSITIVITY, ISO_VALUE);
    
  // Set capture camera size (resolution)
  cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
    
  // Open windows on your monitor
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  
  // Create trackbars for white balancing
//   createTrackbar( " Red: ", source_window, &redbalance, 100, whiteBalanceCallback );
//   createTrackbar( " Blue:", source_window, &bluebalance, 100, whiteBalanceCallback );
  
  // Create trackbar for exposure setting
//   createTrackbar( " Exposure:", source_window, &exposure, 100, NULL);
  
  // set the callback function for any mouse event
  setMouseCallback(source_window, mouseCallback, NULL);
  
  // set default white balance
  whiteBalanceCallback(0,0);
  
  
  // Start multithreading!
  pthread_t cameraThread;
  
  // Setting Priorities
  pthread_attr_t tattr;
  sched_param param;
  
  // Initialize attributes with defaults
  pthread_attr_init (&tattr);
  // Save the parameters to "param"
  pthread_attr_getschedparam (&tattr, &param);
  // Set the priority parameter of "param", leaving others at default
  param.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
  // Set attributes to modified parameters
  pthread_attr_setschedparam (&tattr, &param);

  // Create thread using modified attributes
  pthread_create (&cameraThread, &tattr, takePictures, NULL);
  
  
  // Pause for 2 seconds to let everything initialize
  sleep(2);
  
  // Announce that we're done initializing
  cout << "Done Initializing." << endl;
  
  int key = 0; 
  
  // Take a bunch of pictures
  while(key != 27) // 27 is keycode for escape key
//   for(int i=1; i<=10; i++)
  { 
    // Save most recent frame to local variable
    frame.copyTo(bgrframe);
    
    // Convert color spaces
    cvtColor(bgrframe, yuvframe, CV_BGR2YCrCb);
    
    // Display image on current open window
    imshow( sourcewindow, bgrframe );

   // wait 1 ms to check for press of escape key
   key = waitKey(1);
  } // end while
  
  // Set mode to STOPPED
  stopprogram = 1;
  
  // Wait a second to let the threads stop
  sleep(1);
  
  // close camera
  cap.release();
  picamctrl.close();
}// end main
