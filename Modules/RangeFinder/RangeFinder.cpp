/*****************************************************************************
 * RangeFinder.cpp
 *
 * Module: RangeFinder
 * 
 * Take images and identify laser dot to determine range
 * 
 ******************************************************************************/

#include "RangeFinder.h"

using namespace cv;
using namespace std;

//////////////////////
// Global Variables //
//////////////////////

// For writing image to screen
local_images_t localimages;
cv::Mat hsvframe;
cv::Point p1;


/*******************************************************************************
 * void *rangeFinder(void*)
 * 
 * Calculate range to the bottom and save in a global state variable
 ******************************************************************************/
void *rangeFinder(void*)
{
  int cropwidth = 0.3*FRAME_WIDTH;
  int cropheight = FRAME_HEIGHT - 1;
  int cropleft = (FRAME_WIDTH - cropwidth)/2;
  int croptop = 0;
  Rect roi(cropleft,croptop,cropwidth,cropheight);
  
  // Sleep to prevent race conditions with locks
  auv_msleep(100);
  
  // Take a bunch of pictures
  while(substate.mode != STOPPED) 
  { 
    #ifdef DEBUG
    printf("%s\n","rangeFinder thread" );
    #endif
    
    // Lock subimages.brightframe
    pthread_mutex_lock(&subimages.brightframelock);
    
    // Save most recent bright frame to local variables
    subimages.brightframe.copyTo(localimages.brightframe);
    
    // Unlock subimages.brightframe
    pthread_mutex_unlock(&subimages.brightframelock);
    
    // Lock subimages.darkframe
    pthread_mutex_lock(&subimages.darkframelock);
    
    // Save most recent dark frame to local variables
    subimages.darkframe(roi).copyTo(localimages.darkframe);
    
    // Unlock subimages.darkframe
    pthread_mutex_unlock(&subimages.darkframelock);
    
    // only write this if we're in DEBUG mode
    #ifdef DEBUG
    cout << "Brightframe size= " << localimages.brightframe.size()<< endl;
    cout << "Darkframe size= " << localimages.darkframe.size()<< endl;
    #endif

    // Convert from BGR to HSV using CV_BGR2HSV conversion
    cvtColor(localimages.darkframe, hsvframe, CV_BGR2HSV);
             
    // Find laser dot
    Mat1b mask; // b/w matrix for laser detection
    
    // White
  // inRange(hsvframe, Scalar(0, 0, 40), Scalar(180, 255, 255), mask);
  
    // Green
   // inRange(hsvframe, Scalar(61, 50, 50), Scalar(89, 250, 250), mask);
   // inRange(hsvframe, Scalar(60, 50, 50), Scalar(89, 255, 255), mask);
    
    // Red
    Mat1b mask1, mask2;
    inRange(hsvframe, Scalar(1, 50, 50), Scalar(15, 250, 250), mask1);
    inRange(hsvframe, Scalar(165, 50, 50), Scalar(179, 250, 250), mask2);
    mask = mask1 | mask2;    
    
    // Locate centroid of laser dot
    Moments m = moments(mask, false);
    Point p1(m.m10/m.m00+cropleft, m.m01/m.m00);
    
    // Draw red circle w/ center at centroid on original image
    circle( localimages.brightframe, p1, 10.0, Scalar( 0, 0, 255 ));
    
    // Write coordinates in top left corner
    stringstream coordinates;
    coordinates << "(" << p1.x << ", " << p1.y << ")";
    Point org;
    org.x = 10;
    org.y = 40;
    putText( localimages.brightframe, coordinates.str(), org, 1, 1, Scalar(0,0,255));
    
    // Calculate range
    stringstream rangestring;
    if(p1.x !=0)
    {
      float range = (RANGE_K0 + RANGE_K1*p1.y) / (RANGE_K2 + p1.y);
      substate.range = range;
      cout << "Calculated Range: " << substate.range << endl;
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
    putText( localimages.brightframe, rangestring.str(), pnt, 1, 1, Scalar(0,0,255));
    
    auv_msleep(1000/RANGE_RATE);
   } // end while
        
   pthread_exit(NULL);
}