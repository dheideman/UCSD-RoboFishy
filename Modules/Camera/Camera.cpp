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

//////////////////////
// Global Variables //
//////////////////////

// Image Capture Object
cv::VideoCapture cap;

// V4L2Control Object
V4L2Control picamctrl;

// Image Storage
sub_images_t subimages;

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
  
  // Lock the images until we can collect them
  pthread_mutex_lock(&subimages.brightframelock);
  pthread_mutex_lock(&subimages.darkframelock);
  
  // Let people know we don't have any images for them
  subimages.imstate = EMPTY; 
  
  // Set framerate (OpenCV capture property)
  cap.set(CV_CAP_PROP_FPS,FRAME_RATE);
    
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
  
  // Now that we have the images, we can unlock them
  pthread_mutex_unlock(&subimages.brightframelock);
  pthread_mutex_unlock(&subimages.darkframelock);

  // Grab all 5 images from the frame buffer in order to clear the buffer
  for(int i=0; i<5; i++)
  {
    cap.grab();
  }
  printf("%s\n","BUFFER CLEARED" );

  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
    /* Bright */
    
    // 'Grab' bright frame from webcam's image buffer
    cap.grab();
    
    // Set exposure now (rather than later)
    picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, DARK_EXPOSURE );
    
    // Turn laser on before grabbing the darkframe image
    if(substate.laserarmed == ARMED)
    {
      printf("Laser %d ON!\n",LASERPIN);
      digitalWrite(LASERPIN, HIGH);
    }
    
    // Lock access to subimages.brightframe
    pthread_mutex_lock(&subimages.brightframelock);
    
    // Retrieve encodes image from grab buffer to 'brightframe' variable
    cap.retrieve( subimages.brightframe );
    subimages.imstate = BRIGHTFRAME;
    
    // Unlock access to subimages.brightframe
    pthread_mutex_unlock(&subimages.brightframelock);
    
    // Put in some sleep time just in case
    auv_msleep(100/FRAME_RATE);
    
    /* Dark */

    // 'Grab' dark frame from webcam's image buffer
    cap.grab();
    
    // Turn laser off                               Check the timing on this...
    printf("Laser off...\n");
    digitalWrite(LASERPIN, LOW);
    
    // Set exposure now (rather than later)
    picamctrl.set(V4L2_CID_EXPOSURE_ABSOLUTE, BRIGHT_EXPOSURE );
    
    // Lock access to subimages.darkframe
    pthread_mutex_lock(&subimages.darkframelock);
    
    // Retrieve encodes image from grab buffer to 'darkframe' variable
    cap.retrieve( subimages.darkframe );
    // Let the computer know which frame its at 
    subimages.imstate = DARKFRAME;
    
    // Unlock access to subimages.darkframe
    pthread_mutex_unlock(&subimages.darkframelock);
    
    // Put in some sleep time just in case
    auv_msleep(100/FRAME_RATE);
  }
    
  // close camera
  cap.release();
  picamctrl.close();
  
  // End thread
  pthread_exit(NULL);
}


/*******************************************************************************
 * int initializeSubImagesLock(sub_images_t *_subimages)
 *
 * Initialize locks for sub_images_t variable
 ******************************************************************************/
int initializeSubImagesLock(sub_images_t *_subimages)
{
  if (pthread_mutex_init(&_subimages->brightframelock, NULL) != 0)
  {
    cout << "Unable to initialize brightframelock" << endl;
    return 0;
  }
  if (pthread_mutex_init(&_subimages->darkframelock, NULL) != 0)
  {
    cout << "Unable to initialize darkframelock" << endl;
    return 0;
  }
  return 1;
}

/*******************************************************************************
 * void *writeImages(void*)
 * 
 * Camera-handling thread: continuously save images as soon as they come in
 ******************************************************************************/
void *writeImages(void*)
{
  int i = 0;
  // Loop quickly to pick up images as soon as they are taken
  while(substate.mode != STOPPED)
  {
    // Open data output file:
    ofstream datafile;
    datafile.open (CSV_FILENAME);

  	// If we exceed pitch or roll setpoints disarm the laser as to not blind anyone
    if( subimages.imstate == BRIGHTFRAME )
    {
      // Create filename
      stringstream filename;
      filename << IMAGE_PREFIX;
      if(i<10)    filename << "0";   // add in a zero to 1-digit numbers
      if(i<100)   filename << "0";   // add in a 2-digit numbers
      if(i<1000)  filename << "0";   // add in a 3-digit numbers
      if(i<10000) filename << "0";   // add in a 4-digit numbers
      filename << i << IMAGE_EXTENSION;

      // Lock brightframe
      pthread_mutex_lock(&subimages.brightframelock);
      
      // Write image to file
      imwrite(filename.str(), subimages.brightframe);
      
      // Unlock brightframe
      pthread_mutex_lock(&subimages.brightframelock);
      
      // Increment the image number
      i++;
    }
    // close csv file
    datafile.close();
    
    auv_usleep(100000);
  }
  pthread_exit(NULL);
}