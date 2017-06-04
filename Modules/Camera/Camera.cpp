/*****************************************************************************
 * Camera.cpp
 *
 * Module: Camera
 * 
 * Pull images from camera as soon as they are taken and store them safely
 * 
 ******************************************************************************/

#include "Camera.h"

using namespace cv;
using namespace std;
 /*******************************************************************************
 * void *takePictures(void*)
 * 
 * Camera-handling thread: continuously save images as soon as they come in
 ******************************************************************************/
void *takePictures(void*)
{
  // Open the camera
  cap.open(0);  
  // opens first video device
  picamctrl.open("/dev/video0");

  // check to make sure device properly opened
  if ( !cap.isOpened() )
  {
    cerr << "Error opening the camera (OpenCV)" << endl; 
  }
 
  // Set framerate (OpenCV capture property)
  cap.set(CV_CAP_PROP_FPS,2);
    
  // Set camera exposure, white balance, and iso control to manual (driver property)
  picamctrl.set(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  picamctrl.set(V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,V4L2_WHITE_BALANCE_MANUAL);
  picamctrl.set(V4L2_CID_ISO_SENSITIVITY_AUTO, 0);
  
  // Initialize exposure, iso values
  picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE);
  picamctrl.set(V4L2_CID_ISO_SENSITIVITY, ISO_VALUE);
    
  // Set capture camera size (resolution)
  cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
  
  // Fill images w/ initial images
  cap.read( subimages.brightframe );
  cap.read( subimages.darkframe );

  // Grab all 5 images from the frame buffer in order to clear the buffer
  //printf("%s\n","Before For Loop" );
  for(int i=0; i<5; i++)
  {
    cap.grab();
  }
  printf("%s\n","BUFFER CLEARED" );
  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
    // 'Grab' bright frame from webcam's image buffer
    cap.grab();
    
    // Set exposure now (rather than later)
    picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, DARK_EXPOSURE );
    
    // Turn laser on
    digitalWrite(LASERPIN, HIGH);
    
    // Retrieve encodes image from grab buffer to 'brightframe' variable
    cap.retrieve( subimages.brightframe );
    
  //////
    
    // 'Grab' dark frame from webcam's image buffer
    cap.grab();
    
    // Turn laser off
    digitalWrite(LASERPIN, LOW);
    
    // Set exposure now (rather than later)
    picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE );
    
    // Retrieve encodes image from grab buffer to 'darkframe' variable
    cap.retrieve( subimages.darkframe );

    usleep(200000);
  }
    
  // close camera
  cap.release();
  picamctrl.close();
  
  // End thread
  pthread_exit(NULL);
}