/******************************************************************************
 * FeatureTracker.cpp
 * 
 * Try to track linear position using camera...  This can't end well...
 * 
 * Person on the Internet:
 * http://stackoverflow.com/questions/31835114/feature-detection-with-patent-free-descriptors
 *
 * FLANN: 
 * http://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html?highlight=flannbasedmatcher#flannbasedmatcher
 ******************************************************************************/

#include "FeatureTracker.h"

using namespace cv;
using namespace std;


//////////
// Main //
//////////
int main(int argc, char** argv)
{
  if( argc < 2 )
  {
    cout << " Usage: ./FeatureTracker <videofeed>" << endl;
    return -1;
  }
  
  // Fire up the camera
  VideoCapture cap(atoi(argv[1]));
  
  cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
  
  // Clear buffer
  for(int i=0; i<5; i++)
  {
    cap.grab();
//     imagegrabbed
  }
  cap.retrieve(subimages.brightframe);
  
  // Initialize RNG
  RNG rng(12345);

  // Create point arrays for image corners
  vector<Point2f> newcorners(4);
  vector<Point2f> oldcorners(4);
  
  // Create array of corner points of object image
  newcorners[0] = cvPoint(0, 0);
  newcorners[1] = cvPoint(FRAME_WIDTH, 0);
  newcorners[2] = cvPoint(FRAME_WIDTH, FRAME_HEIGHT);
  newcorners[3] = cvPoint(0, FRAME_HEIGHT);
  
  // Create matrix to store final output image
  Mat matchesimg(Size(2*FRAME_WIDTH,FRAME_HEIGHT),CV_8UC3);
  
  // Initialize Mutex locks
  if(!initializeOdomDataLock(&odomdata)) return -1;
  if(!initializeSubImagesLock(&subimages)) return -1;
  
  
  // Initialize priority thread attributes
  initializeTAttr();
  
  // Thread handles
  pthread_t odometryThread;

  // Create threads using modified attributes
  pthread_create (&odometryThread, &tattrhigh, visualOdometry, NULL);
  
  // Destroy the thread attributes
  destroyTAttr()
  
  
  // Wait for stuff to initialize
  sleep(1);
  
  // MAIN LOOP!
  while (waitKey(1) != 27)
  {
    // Lock subimages.brightframe
    pthread_mutex_lock(&subimages.brightframelock);
    
    // Get picture
    cap.read(subimages.brightframe);
    
    // Unlock subimages.brightframe
    pthread_mutex_unlock(&subimages.brightframelock);
    
    // Transform corner points by tf to get corresponding points on scene image
    perspectiveTransform(newcorners, oldcorners, odomdata.tf);
    
    
    // Lock odomdata.newimg, odomdata.oldimg
    pthread_mutex_lock(&odomdata.newimglock);
    pthread_mutex_lock(&odomdata.oldimglock);
    
    // Combine images onto image of matches (matchesimg)
    Mat left(matchesimg, Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT));
    odomdata.newimg.copyTo(left);
    Mat right(matchesimg, Rect(FRAME_WIDTH, 0, FRAME_WIDTH, FRAME_HEIGHT));
    odomdata.oldimg.copyTo(right);
    
    // Unlock odomdata.newimg, odomdata.oldimg
    pthread_mutex_unlock(&odomdata.newimglock);
    pthread_mutex_unlock(&odomdata.oldimglock);
    
    // Draw matches
    for(int i=0; i<odomdata.oldpts.size(); i++)
    {
      // Generate a random color
      Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255),
                            rng.uniform(0, 255));
      
      // Calculate position of pixel in old image
      Point2f shiftoldpt = odomdata.oldpts[i] + Point2f(FRAME_WIDTH, 0);
      
      // Draw circles at each point
      circle(matchesimg, odomdata.newpts[i], MATCH_CIRCLE_R, color, 1, 8, 0);
      circle(matchesimg, shiftoldpt, MATCH_CIRCLE_R, color, 1, 8, 0);
      
      // Draw a connecting line between the matches
      line(matchesimg, odomdata.newpts[i], shiftoldpt, color, 1);
    }

    // Draw box around detected object in scene image, one line at a time
    line(matchesimg, oldcorners[0] + Point2f(FRAME_WIDTH, 0),
         oldcorners[1] + Point2f(FRAME_WIDTH, 0), Scalar(0, 255, 0), 4);
    line(matchesimg, oldcorners[1] + Point2f(FRAME_WIDTH, 0),
         oldcorners[2] + Point2f(FRAME_WIDTH, 0), Scalar(0, 255, 0), 4);
    line(matchesimg, oldcorners[2] + Point2f(FRAME_WIDTH, 0),
         oldcorners[3] + Point2f(FRAME_WIDTH, 0), Scalar(0, 255, 0), 4);
    line(matchesimg, oldcorners[3] + Point2f(FRAME_WIDTH, 0),
         oldcorners[0] + Point2f(FRAME_WIDTH, 0), Scalar(0, 255, 0), 4);  
    
    // Show detected matches
    imshow("Good Matches & Object detection", matchesimg);
    usleep(10*1000);
  }
  return 0;
}