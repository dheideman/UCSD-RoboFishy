/*****************************************************************************
 * PoCRangefinder_update.cpp
 *
 * Take images at varying distances from target to calibrate laser stick
 *
 ******************************************************************************/

#include "PoCRangefinder.h"

using namespace cv;
using namespace std;

////////////////
// Parameters //
////////////////

// Saved image params
#define IMAGE_PREFIX    "images/image"
#define IMAGE_EXTENSION ".jpg"

#define BRIGHT_EXPOSURE 100
#define DARK_EXPOSURE   5
#define ISO_VALUE       1

// Image Size
//#define FRAME_WIDTH     3280  // 8 megapixels
//#define FRAME_HEIGHT    2464  // 8 megapixels
#define FRAME_WIDTH       1280  // 720 HD
#define FRAME_HEIGHT      720   // 720 HD

// Rangefinder Constants
#define RANGE_K0          -567.7
#define RANGE_K1          0.09943
#define RANGE_K2          -330.0


//////////////////////
// Global Variables //
//////////////////////

// Global State Variable
sub_state_t substate;

// Global Variables
string source_window = "Bright Image";
Mat darkframe, brightframe;
VideoCapture cap;
int redbalance = 20;
int bluebalance = 20;
int exposure = 5;
int brightness = 50;
int contrast = 50;
int saturation = 50;
Mat hsv_frame;

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
    Vec3b intensity = hsv_frame.at<Vec3b>(y, x);
    int hue = intensity.val[0];
    int sat = intensity.val[1];
    int val = intensity.val[2];
    // print out HSV values (sanity check)
    cout << "H: " << hue << ",\tS:" << sat << ",\tV:" << val << endl;
    
    Vec3b bgr = brightframe.at<Vec3b>(y, x);
    int b = bgr.val[0];
    int g = bgr.val[1];
    int r = bgr.val[2];
    // print out RGB values (sanity check)
    cout << "B: " << b << ",\tG:" << g << ",\tR:" << r << endl;
  }
}

/*******************************************************************************
 * void bcsCallback(int, void*)
 * 
 * Sets brightness, contrast and saturation values (1-100)
 ******************************************************************************/
void bcsCallback(int, void*)
{
  picamctrl.set(V4L2_CID_BRIGHTNESS,brightness);
  picamctrl.set(V4L2_CID_CONTRAST,contrast);
  picamctrl.set(V4L2_CID_SATURATION,saturation);
  // sanity checks: print out brightness, contrast, and saturation values
  cout << "B: " << picamctrl.get(V4L2_CID_BRIGHTNESS) << "\t";
  cout << "C: " << picamctrl.get(V4L2_CID_CONTRAST) << "\t";
  cout << "S: " << picamctrl.get(V4L2_CID_SATURATION) << endl;
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
  cap.open(0); // opens first video device
  picamctrl.open("/dev/video0");

  // check to make sure device properly opened
  if ( !cap.isOpened() )
  {
    cerr << "Error opening the camera (OpenCV)" << endl;
    return -1;
  }
 // MOVE THESE TO THE CAMERA THREAD 
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
  
  // Fill images w/ initial images
  cap.read( brightframe );
  cap.read( darkframe );
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
  
  // Fill images w/ initial images
  cap.read( brightframe );
  cap.read( darkframe );
  // Initialize exposure settings
  picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE );
  
  // Grab all 5 images from the frame buffer in order to clear the buffer
  for(int i=0; i<5; i++)
  {
    cap.grab();
  }
  
  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
    // 'Grab' bright frame from webcam's image buffer
    cap.grab();
    // Set exposure now (rather than later)
    picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, DARK_EXPOSURE );
    // Retrieve encodes image from grab buffer to 'brightframe' variable
    cap.retrieve( brightframe );
    
    // 'Grab' dark frame from webcam's image buffer
    cap.grab();
    // Set exposure now (rather than later)
    picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE );
    // Retrieve encodes image from grab buffer to 'darkframe' variable
    cap.retrieve( darkframe );
  }
  
  // End thread
  pthread_exit(NULL);
}


//////////
// Main //
//////////
int main(int argc, char** argv)
{ 
  // We're initializing!
  substate.mode = INITIALIZING;
  
  // Open the camera!
    
  // Open windows on your monitor
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  
  // Create trackbars for white balancing
//   createTrackbar( " Red: ", source_window, &redbalance, 100, whiteBalanceCallback );
//   createTrackbar( " Blue:", source_window, &bluebalance, 100, whiteBalanceCallback );
  
  // Create trackbar for exposure setting
//   createTrackbar( " Exposure:", source_window, &exposure, 100, NULL);
  
  // Create brightness, saturation and contrast trackbars
//   createTrackbar( " Brightness:", source_window, &brightness, 100, bcsCallback);
//   createTrackbar( " Contrast:", source_window, &contrast, 100, bcsCallback);
//   createTrackbar( " Saturation:", source_window, &saturation, 100, bcsCallback);
  
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
  
  // Set mode to RUNNING
  substate.mode = RUNNING;
  
  int key = 0; 
  
  // Take a bunch of pictures
  while(key != 27) // 27 is keycode for escape key
//   for(int i=1; i<=10; i++)
  { 
    // Save most recent bright frame to local variables
    Mat localbrightframe, localdarkframe;
    brightframe.copyTo(localbrightframe);
    darkframe.copyTo(localdarkframe);
    
    // Convert from BGR to HSV using CV_BGR2HSV conversion
//    Mat hsv_frame;
    cvtColor(localdarkframe, hsv_frame, CV_BGR2HSV);
    
    // Find laser dot
    Mat1b mask; // b/w matrix for laser detection
    
    // if hsv_frame is in scalar range, set that pixel to white and store in mask
    
    // White
//     inRange(hsv_frame, Scalar(0, 0, 40), Scalar(180, 255, 255), mask);
    
    // Green
    inRange(hsv_frame, Scalar(61, 50, 50), Scalar(89, 250, 250), mask);
    
    // Red
//    Mat1b mask1, mask2;
//    inRange(hsv_frame, Scalar(1, 50, 50), Scalar(15, 250, 250), mask1);
//    inRange(hsv_frame, Scalar(165, 50, 50), Scalar(179, 250, 250), mask2);
//    mask = mask1 | mask2;    
    
    // Locate centroid of laser dot
    Moments m = moments(mask, false);
    Point p1(m.m10/m.m00, m.m01/m.m00);
    
    // Draw red circle w/ center at centroid on original image
    circle( localbrightframe, p1, 10.0, Scalar( 0, 0, 255 ));
    

    // Write coordinates in top left corner
    stringstream coordinates;
    coordinates << "(" << p1.x << ", " << p1.y << ")";
    Point org;
    org.x = 10;
    org.y = 40;
    putText( localbrightframe, coordinates.str(), org, 1, 1, Scalar(0,0,255));
    
    // Calculate range
    stringstream rangestring;
    if(p1.x !=0)
    {
      float range = (RANGE_K0 + RANGE_K1*p1.y) / (RANGE_K2 + p1.y);
//       cout << "Calculated Range: " << range << endl;
      rangestring << range << " ft";
    }
    else  // If coordinates are (0,0), we haven't detected anything.
    {
      rangestring << "------ ft";
    }
    
    // Write range in top left corner
    Point pnt;
    pnt.x = 10;
    pnt.y = 20;
    putText( localbrightframe, rangestring.str(), pnt, 1, 1, Scalar(0,0,255));
    
    // Display image on current open window
    imshow( source_window, localbrightframe );
    //     imshow( source_window, mask );

   // wait 1 ms to check for press of escape key
   key = waitKey(1);
  } // end while
  
  // Set mode to STOPPED
  substate.mode = STOPPED;
  
  // Wait a second to let the threads stop
  sleep(1);
  
  // close camera
  cap.release();
  picamctrl.close();
}// end main
