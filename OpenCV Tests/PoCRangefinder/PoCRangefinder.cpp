/*****************************************************************************
 * PoCRangefinder.cpp
 *
 * Take images at varying distances from target to calibrate laser stick
 *
 ******************************************************************************/

#include "PoCRangefinder.h"

////////////////
// Parameters //
////////////////

// Saved image params
#define IMAGE_PREFIX    "images/image"
#define IMAGE_EXTENSION ".jpg"

#define BRIGHT_EXPOSURE 100
#define DARK_EXPOSURE   1 //exposure

// Image Size
//#define FRAME_WIDTH     3280  // 8 megapixels
//#define FRAME_HEIGHT    2464  // 8 megapixels
#define FRAME_WIDTH       1280  // 720 HD
#define FRAME_HEIGHT      720   // 720 HD


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
int exposure = 10;
int brightness = 50;
int contrast;
int saturation;
Mat hsv_frame;

// V4L2 Global Variables
int fd;


///////////////////
// V4L2 Controls //
///////////////////

/*******************************************************************************
 * void writeV4L2Error(int fd, unsigned int id, int value)
 * 
 * Write to terminal the details of an error given the id trying to be changed
 * and the value it was to be set to.
 ******************************************************************************/
void writeV4L2Error(int fd, unsigned int id, int value)
{
  struct v4l2_querymenu qmenu;
  struct v4l2_queryctrl qctrl;
  
  string ctrlname;
  string menuvalue;
  int namelength;
  
  // make sure qctrl is cleared
  memset(&qctrl, 0, sizeof(qctrl));
  
  // Find the control that matches the requested id
  qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  while ((ctrlname.size()==0) && (0==ioctl (fd, VIDIOC_QUERYCTRL, &qctrl)))
  { 
    if( qctrl.id == id)
    {
      // Save the name of the control (as a string)
      namelength = sizeof qctrl.name / sizeof qctrl.name[0];
      ctrlname = string(qctrl.name, qctrl.name + namelength);
      
      // Check if this control is a menu type
      if (qctrl.type == V4L2_CTRL_TYPE_MENU)
      {
        memset(&qmenu, 0, sizeof(qmenu));
        qmenu.id = qctrl.id;
        
        // Loop through the menu items
        for (qmenu.index = qctrl.minimum;
             qmenu.index <= qctrl.maximum;
             qmenu.index++)
        {
          if (0 == ioctl(fd, VIDIOC_QUERYMENU, &qmenu))
          {
            if(qmenu.id == value)
            {
              namelength = sizeof qmenu.name / sizeof qmenu.name[0];
              menuvalue = string(qmenu.name, qmenu.name + namelength);
            }
          }
        } // end for (qmenu.index...)
      } // end if(qctrl.type...)        
    } // end if( qctrl.id...)
    
    // Move to next id.
    qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  } // end while
  
  // Write out the error message
  // if a name was found for the value setting:
  if (menuvalue.size() != 0)
    cout << "Could not set \"" << ctrlname << "\" to " << menuvalue << endl;
  // otherwise just use the value given to us
  else
    cout << "Could not set \"" << ctrlname << "\" to " << value << endl;
    
  // Print out error code:
  switch (errno)
  {
    case EINVAL:
      cout << "Error Code: EINVAL" << endl;
      break;
    case ERANGE:
      cout << "Error Code: ERANGE" << endl;
      break;
    case EBUSY:
      cout << "Error Code: EBUSY" << endl;
      break;
    case EACCES:
      cout << "Error Code: EACCES" << endl;
      break;
  }
}

/*******************************************************************************
 * int setV4L2Control(int fd, unsigned int id, int value)
 * 
 * Set the V4L2 control "id" for device "fd" to "value"
 ******************************************************************************/
int setV4L2Control(int fd, unsigned int id, int value)
{
  struct v4l2_control control;
  
  memset(&control, 0, sizeof (control));
  control.id = id;
  control.value = value;
  if (-1 == ioctl(fd, VIDIOC_S_CTRL, &control))
  {
    writeV4L2Error(fd, id, value);
    return -1;
  }
  else return 0;
}

/*******************************************************************************
 * int getV4L2Control(int fd, unsigned int id)
 * 
 * Returns the current value of the V4L2 control "id" for device "fd"
 ******************************************************************************/
int getV4L2Control(int fd, unsigned int id)
{
  struct v4l2_control control;
  
  memset(&control, 0, sizeof (control));
  control.id = id;
  if (-1 == ioctl(fd, VIDIOC_G_CTRL, &control)) return -1;
  else return control.value;
}


////////////////////////
// Callback Functions //
////////////////////////

/*******************************************************************************
 * void whiteBalanceCallback(int, void*)
 * 
 * Sets white balance values
 ******************************************************************************/
void whiteBalanceCallback(int, void*)
{
  setV4L2Control(fd, V4L2_CID_RED_BALANCE,redbalance*80);
  setV4L2Control(fd, V4L2_CID_BLUE_BALANCE,bluebalance*80);
}

/*******************************************************************************
 * void mouseCallback(int event, int x, int y, int flags, void* userdata)
 * 
 * Write HSV and RGB values of point clicked on to terminal
 ******************************************************************************/
void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == EVENT_LBUTTONDOWN )
  {
    Vec3b intensity = hsv_frame.at<Vec3b>(y, x);
    int hue = intensity.val[0];
    int sat = intensity.val[1];
    int val = intensity.val[2];
    cout << "H: " << hue << ",\tS:" << sat << ",\tV:" << val << endl;
    
    Vec3b bgr = brightframe.at<Vec3b>(y, x);
    int b = bgr.val[0];
    int g = bgr.val[1];
    int r = bgr.val[2];
    cout << "B: " << b << ",\tG:" << g << ",\tR:" << r << endl;
  }
}

/*******************************************************************************
 * void bcsCallback(int, void*)
 * 
 * Sets brightness, contrast an saturation values
 ******************************************************************************/
void bcsCallback(int, void*)
{
  setV4L2Control(fd, V4L2_CID_BRIGHTNESS,brightness*80);
  setV4L2Control(fd, V4L2_CID_CONTRAST,contrast*80);
  setV4L2Control(fd, V4L2_CID_SATURATION,saturation*80);
  cout << "B: " << getV4L2Control(fd, V4L2_CID_BRIGHTNESS) << "\t";
  cout << "C: " << getV4L2Control(fd, V4L2_CID_CONTRAST) << "\t";
  cout << "S: " << getV4L2Control(fd, V4L2_CID_SATURATION) << endl;
}

//////////////
// Threads! //
//////////////

/*******************************************************************************
 * void *takePictures(void*)
 * 
 * Camera-handling thread: save images as soon as they come in
 ******************************************************************************/
void *takePictures(void*)
{
  // Initialize exposure settings
  setV4L2Control(fd, V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE );
  
  // Clear the frame buffer
  for(int i=0; i<5; i++)
  {
    cap.grab();
  }
  
  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
    // Get bright frame
    cap.grab();
    setV4L2Control(fd, V4L2_CID_EXPOSURE_ABSOLUTE, DARK_EXPOSURE );
    cap.retrieve( brightframe );
    
    // Get dark frame
    cap.grab();
    setV4L2Control(fd, V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE );
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
  cap.open(0);
  fd = open("/dev/video0",  O_RDWR /* required */ | O_NONBLOCK, 0);

  if ( !cap.isOpened() )
  {
    cerr << "Error opening the camera (OpenCV)" << endl;
    return -1;
  }
  
  // Set framerate
  cap.set(CV_CAP_PROP_FPS,2);
  
  
  // Set camera exposure control to manual
  setV4L2Control(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  
  // Set camera awb to manual ?
  setV4L2Control(fd, V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
                     V4L2_WHITE_BALANCE_MANUAL);

  // Set camera iso to manual
  setV4L2Control(fd, V4L2_CID_ISO_SENSITIVITY_AUTO, 0);
  
  // Initialize exposure, iso values
  setV4L2Control(fd, V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE);
  setV4L2Control(fd, V4L2_CID_ISO_SENSITIVITY, 0);
    
  // Set capture camera size
  cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
  
  
  // Fill images w/ initial images
  cap.read( brightframe );
  cap.read( darkframe );
  
  
  // Open windows
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  
  // Create trackbars for white balancing
//   createTrackbar( " Red: ", source_window, &redbalance, 100, whiteBalanceCallback );
//   createTrackbar( " Blue:", source_window, &bluebalance, 100, whiteBalanceCallback );
  
  // Create trackbar for exposure setting
//   createTrackbar( " Exposure:", source_window, &exposure, 100, NULL);
  
  // Create brightness, saturation and contrast trackbars
  createTrackbar( " Brightness:", source_window, &brightness, 100, NULL);
  createTrackbar( " Contrast:", source_window, &contrast, 100, NULL);
  createTrackbar( " Saturation:", source_window, &saturation, 100, NULL);
  
  //set the callback function for any mouse event
  setMouseCallback(source_window, mouseCallback, NULL);
  
  //set default white balance
  whiteBalanceCallback(0,0);
  
  
  // Start multithreading!
  pthread_t cameraThread;
  pthread_create (&cameraThread, NULL, takePictures, NULL);
  
  
  // Pause for 2 seconds to let everything initialize
  sleep(2);
  
  // Announce that we're done initializing
  cout << "Done Initializing." << endl;
  
  // Set mode to RUNNING
  substate.mode = RUNNING;
  
//  char keyin = '0';
  int key = 0; 
  
  // Take a bunch of pictures
 while(key != 27)
//   for(int i=1; i<=10; i++)
  { 
    // Save most recent bright frame
    Mat laserframe, darklaserframe;
    brightframe.copyTo(laserframe);
    darkframe.copyTo(darklaserframe);
    
    // Convert from BGR to HSV
//    Mat hsv_frame;
    cvtColor(darklaserframe, hsv_frame, CV_BGR2HSV);
    
    // Find laser dot
    Mat mask;
//     inRange(hsv_frame, Scalar(0, 0, 40), Scalar(180, 255, 255), mask);
    inRange(hsv_frame, Scalar(61, 50, 50), Scalar(89, 250, 250), mask);
    
    // Locate centroid of laser dot
    Moments m = moments(mask, false);
    Point p1(m.m10/m.m00, m.m01/m.m00);
    
    // Draw circle w/ center at centroid on original image
    circle( laserframe,p1,10.0,Scalar( 0, 0, 255 ));
    

    // Write coordinates in top left corner
    stringstream coordinates;
    coordinates << "(" << p1.x << ", " << p1.y << ")";
    Point org;
    org.x = 10;
    org.y = 20;
    putText( laserframe, coordinates.str(), org, 1, 1, Scalar(0,0,255));
    
    // Create filename
//     stringstream filename;
//     filename << IMAGE_PREFIX;
//     if(i<10) filename << "0";   // add in a zero to 1-digit numbers
//     filename << i << IMAGE_EXTENSION;
 
    // Write image to file
//     imwrite(filename.str(), laserframe);
    imshow( source_window, laserframe );
//     imshow( source_window, mask );

    // Print white balance to screen
//     cout << "Red:  " << getV4L2Control(fd, V4L2_CID_RED_BALANCE) << endl;
//     cout << "Blue: " << getV4L2Control(fd, V4L2_CID_BLUE_BALANCE) << endl;
    
    // check for press of enter key
   key = waitKey(1);
  } // end while
  
  // Set mode to STOPPED
  substate.mode = STOPPED;
  
  // Wait a second to let the threads stop
  sleep(1);
  
  // close camera
  cap.release();
  close(fd);
}// end main
