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
  // Take a bunch of pictures
  while(substate.mode != STOPPED) 
//   for(int i=1; i<=10; i++)
  { 
    printf("%s\n","rangeFinder thread" );
    // Save most recent bright frame to local variables
    //Mat localbrightframe, localdarkframe;
    brightframe.copyTo(localimages.brightframe);
    darkframe(roi).copyTo(localimages.darkframe);
  cout << "Brightframe size= " << brightframe.size()<< endl;
  cout << "Darkframe size= " << darkframe.size()<< endl;

    // Convert from BGR to HSV using CV_BGR2HSV conversion
    //cvtColor(localdarkframe, hsvframe, CV_BGR2HSV);
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
    //p1(m.m10/m.m00+cropleft, m.m01/m.m00);
    
    // Draw red circle w/ center at centroid on original image
    //circle( localbrightframe, p1, 10.0, Scalar( 0, 0, 255 ));
    circle( localimages.brightframe, p1, 10.0, Scalar( 0, 0, 255 ));
    
    // Write coordinates in top left corner
    stringstream coordinates;
    coordinates << "(" << p1.x << ", " << p1.y << ")";
    Point org;
    org.x = 10;
    org.y = 40;
    //putText( localbrightframe, coordinates.str(), org, 1, 1, Scalar(0,0,255));
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
    //putText( brightframeroi, rangestring.str(), pnt, 1, 1, Scalar(0,0,255));
    
    // Display image on current open window
    sleep(1);
    // imshow( SOURCE_WINDOW, localbrightframe );
    // imshow( SOURCE_WINDOW, darkframeroi );
    // imshow( SOURCE_WINDOW, mask );
   } // end while
        
   pthread_exit(NULL);
}