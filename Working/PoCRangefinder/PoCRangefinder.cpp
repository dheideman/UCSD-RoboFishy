/****************************************************************************
 * PoCRangefinder.cpp
 *
 * Draw what the rangefinder "sees" to the screen
 * - Moved all camera settings and capture into thread
 ******************************************************************************/

#include "PoCRangefinder.h"

using namespace cv;
using namespace std;


////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Global Variables //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Global Variables
int redbalance = 20;
int bluebalance = 20;
int exposure = 5;


////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Callback Functions ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * void whiteBalanceCallback(int, void*)
 * 
 * Sets white balance values (from 1-7999)
 * Changing position of trackbar calls this function
 ******************************************************************************/
void whiteBalanceCallback(int, void*)
{
  picamctrl.set(V4L2_CID_RED_BALANCE,redbalance*80);
  picamctrl.set(V4L2_CID_BLUE_BALANCE,bluebalance*80);
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
   // How do we know what we're looking at?
    Vec3b intensity = hsvframe.at<Vec3b>(y, x);
    int hue = intensity.val[0];
    int sat = intensity.val[1];
    int val = intensity.val[2];
    // print out HSV values (sanity check)
    cout << "H: " << hue << ",\tS:" << sat << ",\tV:" << val << endl;
    
    Vec3b bgr = localimages.darkframe.at<Vec3b>(y, x);
    int b = bgr.val[0];
    int g = bgr.val[1];
    int r = bgr.val[2];
    // print out RGB values (sanity check)
    cout << "B: " << b << ",\tG:" << g << ",\tR:" << r << endl;
  }
}

   
////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// MAIN! //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{ 
  // We're initializing!
  substate.mode = INITIALIZING;
  wiringPiSetup();
  pinMode(LASERPIN, OUTPUT);  
  // Open windows on your monitor
  namedWindow( SOURCE_WINDOW, CV_WINDOW_AUTOSIZE );  
  
  // Create trackbars for white balancing
//   createTrackbar( " Red: ", SOURCE_WINDOW, &redbalance, 100, whiteBalanceCallback );
//   createTrackbar( " Blue:", SOURCE_WINDOW, &bluebalance, 100, whiteBalanceCallback );
  
  // Create trackbar for exposure setting
//   createTrackbar( " Exposure:", SOURCE_WINDOW, &exposure, 100, NULL);
  
  // Create brightness, saturation and contrast trackbars
//   createTrackbar( " Brightness:", SOURCE_WINDOW, &brightness, 100, bcsCallback);
//   createTrackbar( " Contrast:", SOURCE_WINDOW, &contrast, 100, bcsCallback);
//   createTrackbar( " Saturation:", SOURCE_WINDOW, &saturation, 100, bcsCallback);
  
  // set the callback function for any mouse event
  setMouseCallback(SOURCE_WINDOW, mouseCallback, NULL);
  
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
  
  
  // Pause for 4 seconds to let everything initialize
  sleep(4);

  pthread_t rangeThread;

  // Create schedule parameters
  pthread_attr_t rangetattr;
  sched_param rangeparam;
  
  // Initialize attributes with defaults
  pthread_attr_init (&rangetattr);
  // Save the parameters to "param"
  pthread_attr_getschedparam (&rangetattr, &rangeparam);
  // Set the priority parameter of "param", leaving others at default
  rangeparam.sched_priority = sched_get_priority_max(SCHED_RR)/2;
  // Set attributes to modified parameters
  pthread_attr_setschedparam (&rangetattr, &rangeparam);
  // Create the RangeFinder thread
  pthread_create (&rangeThread, &rangetattr, rangeFinder, NULL);
  
 
  // Announce that we're done initializing
  cout << "Done Initializing." << endl;
  // Wait a second to get the rangefinder thread running
  sleep(4); 
  // Set mode to RUNNING
  substate.mode = RUNNING;
  cout << "RUNNING" << endl;

  // initialize key command
  int key = 0; 
  
  //Loop until user presses esc
  while(key != 27)
  {
    imshow( SOURCE_WINDOW, localimages.brightframe );
    key = waitKey(2500);
  }
  
  // Give it time to shut down threads
  sleep(2);

  // Set mode to STOPPED
  substate.mode = STOPPED; 
  cout << "STOPPED." << endl;
  
  
}// end main
