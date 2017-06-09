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

#define EXPOSURE        100
//#define ISO_VALUE       1

// Balancing Region
#define BALANCE_WIDTH     0.2
#define BALANCE_HEIGHT    0.2


//////////////////////
// Global Variables //
//////////////////////

// Global Variables
string sourcewindow = "Current Image";
int redbalance = 1600;
int bluebalance = 1600;
int exposure = 5;
Mat bgrframe;


////////////////////////
// Callback Functions //
////////////////////////

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

//////////
// Main //
//////////
int main(int argc, char** argv)
{
  // Start thread stuff
  initializeTAttr();
  
  pthread_t cameraThread;
  pthread_create (&cameraThread, &tattrhigh, takePictures, NULL);

  destroyTAttr();

  // Open window on your monitor
  namedWindow( sourcewindow, CV_WINDOW_AUTOSIZE );
  
  // set the callback function for any mouse event
  setMouseCallback(sourcewindow, mouseCallback, NULL);
  
  
  // set default white balance
  picamctrl.set(V4L2_CID_RED_BALANCE, redbalance);
  picamctrl.set(V4L2_CID_BLUE_BALANCE, bluebalance);
  
  
  // Calculate region within which we will be balancing the image
  int balanceh_2 = FRAME_HEIGHT * BALANCE_HEIGHT / 2; // half balance box height
  int balancew_2 = FRAME_WIDTH * BALANCE_WIDTH / 2;   // half balance box width
  int framecentery = FRAME_HEIGHT / 2;  // y coord of "center" of frame
  int framecenterx = FRAME_WIDTH / 2;   // x coord of "center" of frame
  
  // Calculate coordinates of balancing box
  int balancexmin = framecenterx - balancew_2;
  int balancexmax = framecenterx + balancew_2 - 1;
  int balanceymin = framecentery - balanceh_2;
  int balanceymax = framecentery + balanceh_2 - 1;
  
  // Generate points for bounds of balancing box
  Point b1 = Point(balancexmin, balanceymin);
  Point b2 = Point(balancexmax, balanceymax);
  
  // Let camera capture some pictures
  sleep(4);

  // Announce that we're done initializing
  cout << "Done Initializing." << endl;
  
  int key = 0; 
  
  // Loop quickly to pick up images as soon as they are taken
  while(key != 27) // 27 is keycode for escape key
  {
    // 'Grab' frame from webcam's image buffer
    subimages.brightframe.copyTo(bgrframe);
    
    // Obtain average BGR values over our balancing box area.
    Scalar bgrmean = mean(bgrframe(Rect(b1, b2)));
    float bbar = bgrmean[0];  // blue
    float rbar = bgrmean[2];  // red
    
    cout << "bbar: " << bbar << "\trbar: " << rbar << endl;
    
    // Check whether blue or red is more off.
    if ( abs(bbar) > abs(rbar) )
    {
      // If red is more wrong, adjust red balance
      redbalance += 0.5*rbar;
      picamctrl.set(V4L2_CID_RED_BALANCE, redbalance);
    }
    else
    {
      // The blue is more wrong, so adjust blue balance
      bluebalance += 0.5*bbar;
      picamctrl.set(V4L2_CID_BLUE_BALANCE, bluebalance);
    }
    
    cout << "Red:  " << redbalance << "\tBlue: " << bluebalance << endl;
    
    // Draw rectangle in middle of image
    rectangle(bgrframe, b1, b2 , Scalar(255, 255, 0));
    
    // Display image on current open window
    imshow( sourcewindow, bgrframe );

   // wait 1 ms to check for press of escape key
   key = waitKey(1);
  } // end while
  
  // Shut down
  substate.mode = STOPPING;

  // Wait for threads to stop
  sleep(4);
}// end main
