/******************************************************************************
 * Odometry.cpp
 * 
 * Track 2D position using down-looking camera.
 * 
 ******************************************************************************/

#include "Odometry.h"

#define DEBUG

using namespace cv;
using namespace std;

//////////////////////
// Global Variables //
//////////////////////

// Global odometry image struct
odom_data_t odomdata;

/*******************************************************************************
 * bool compareDMatch(const DMatch& a, const DMatch& b)
 *
 * Returns true if the distance in match a is smaller than match b.
 * Used for sorting matches smallest to largest
 ******************************************************************************/
bool compareDMatch(const DMatch& a, const DMatch& b)
{
  return a.distance < b.distance;
}


/*******************************************************************************
 * void *visualOdometry(void*)
 * 
 * Visual odometry thread: calculates motion based on camera feed
 ******************************************************************************/
 void *visualOdometry(void* arg)
{
  // Retrieve arguments
//   if (arg != NULL)
//   {
//     // We were passed something, so...  pointers?
//     int* obj = (int*)arg;
//   }
//   // Or maybe just use a global struct containing new, old images and pts?
  
  // Sleep for a little bit to let camera thread lock
  auv_msleep(100);
  
  // We're in!
  cout << "Start Visual Odometry Thread" << endl;
    
  // Lock access to subimages.brightframe, odomdata.oldimg
  pthread_mutex_lock(&subimages.brightframelock);
  pthread_mutex_lock(&odomdata.oldimglock);
  
  // Save bright frame to "old image"
//   subimages.brightframe.copyTo(odomdata.oldimg);
  Size odomframesize = Size(ODOM_FRAME_WIDTH,ODOM_FRAME_HEIGHT);
  resize(subimages.brightframe, odomdata.oldimg, odomframesize);
  
  // Unlock access to subimages.brightframe, odomdata.oldimg
  pthread_mutex_unlock(&odomdata.oldimglock);
  pthread_mutex_unlock(&subimages.brightframelock);
  
  // Done copying in image
  cout << "Done copying 'old' image" << endl;
  
  // Create detector (FAST)
  Ptr<FeatureDetector> detector = FastFeatureDetector::create();
  
  // Create vectors of KeyPoints (features)
  vector<KeyPoint> newkeypoints, oldkeypoints;
  
  // Detect key points of "old" image
  detector->detect(odomdata.oldimg, oldkeypoints);
  
  /* Part 2 Setup: Extract */
  // Create extractor (will characterize each feature)
  Ptr<DescriptorExtractor> extractor = ORB::create();
  
  // Create 2D matrix for descriptors for each image
  // Each row corresponds to a feature
  // 32 columns describe each feature
  Mat newdescriptors, olddescriptors;
  
  // Characterize key points of "old" image
  extractor->compute(odomdata.oldimg, oldkeypoints, olddescriptors);
  
  // Check if there are any descriptors available from the matches  
  if ( olddescriptors.empty() )
  {
    cerr << "Error: olddescriptors is empty" << endl;
//     return -1;
  }
  
  // Make sure the descriptors are in the right type for the FLANN matcher
  if(olddescriptors.type()!=CV_32F)
    olddescriptors.convertTo(olddescriptors, CV_32F);
  
  /* Part 3 Setup: Match */
  // Create FLANN matcher
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
  
  // Create vector of matches, "good" matches
  vector< DMatch > matches, goodmatches;
  
  /* Part 4 Setup: Calculate dx, dy, dtheta */
  // Points to transform to calculate motion
  // [0] = center point
  // [1] = top-center point
  vector<Point2f> pretfpts(2);    // Pre TF Pts
  vector<Point2f> posttfpts(2);   // Post TF Pts
  vector<Point2f> deltatfpts(2);  // Delta TF Pts
  
  // Initialize transformpoints
  pretfpts[0] = cvPoint(FRAME_WIDTH/2, FRAME_HEIGHT/2); // center
  pretfpts[1] = cvPoint(FRAME_WIDTH/2, 0); // center-top
  
  // Declare thread timers
  struct timeval threadstart, now;
  
  while( substate.mode != RUNNING )
  {
    // Get start time of thread
    gettimeofday(&threadstart, NULL);
    
    // Lock access to subimages.brightframe, odomdata.newimg
    pthread_mutex_lock(&subimages.brightframelock);
    pthread_mutex_lock(&odomdata.newimglock);
  
    // Save bright frame to "new image"
//     subimages.brightframe.copyTo(odomdata.newimg);
    resize(subimages.brightframe, odomdata.newimg, odomframesize);
    
    // Unlock access to subimages.brightframe, odomdata.newimg
    pthread_mutex_unlock(&odomdata.newimglock);
    pthread_mutex_unlock(&subimages.brightframelock);
    
    // Part 1: Detect features (KeyPoints)
    detector->detect(odomdata.newimg, newkeypoints);
    
    // Part 2: Characterize each feature, store characteristics in Mats
    extractor->compute(odomdata.newimg, newkeypoints, newdescriptors);
    
    // Make sure we have some descriptors, otherwise return error.
    if ( newdescriptors.empty() )
    {
      cerr << "Error: newdescriptors is empty" << endl;
//       return -1;
    }
    
    // Convert descriptors to CV_32F type for FLANN
    if(newdescriptors.type()!=CV_32F)
      newdescriptors.convertTo(newdescriptors, CV_32F);
      
    // Part 3: Calculate matches between two images
    matcher->match(newdescriptors, olddescriptors, matches);

    /* Draw only "good" matches */
    // Sort matches by distance: shortest to longest
    sort(matches.begin(), matches.end(), compareDMatch);
    
    int nmatches = matches.size();
    int ngoodmatches = (nmatches/2 > MAX_MATCHES ? MAX_MATCHES : nmatches/2);
    
    goodmatches.clear();
    for(int i=0; i<ngoodmatches; i++)
    {
      goodmatches.push_back(matches[i]);
    }
    
    if ( goodmatches.size() == 0 )
    {
      cerr << "Error: no good matches detected" << endl;
//       return -1;
    }
    
    // Part 4: Store all the points corresponding to the good matches
    // Clear vectors first
    odomdata.newpts.clear();
    odomdata.oldpts.clear();
    
    // Now store the good matches
    for (int i = 0; i < goodmatches.size(); i++)
    {
      // Get the keypoints from the good matches
      odomdata.newpts.push_back(newkeypoints[goodmatches[i].queryIdx].pt);
      odomdata.oldpts.push_back(oldkeypoints[goodmatches[i].trainIdx].pt);
    }
     
    // Find perspective transformation between two planes.
    odomdata.tf = findHomography(odomdata.newpts, odomdata.oldpts, CV_RANSAC);
    
    // Transform reference points to determine motion
    perspectiveTransform(pretfpts, posttfpts, odomdata.tf);
    
    // Calculate pure dx, dy, dtheta of points (transformed - initial position)
    deltatfpts[0] = posttfpts[0] - pretfpts[0]; // center
    deltatfpts[1] = posttfpts[1] - posttfpts[0]; // top-center
    
    // Calculate dtheta using top-center motion
    float dtheta = atan2(-1*deltatfpts[1].x, -1*deltatfpts[1].y);
    
    // Calculate new pose    
    float dx = deltatfpts[0].x;
    float dy = deltatfpts[0].y;
    float st = sin(substate.pose.z);    // Using theta as determined by camera 
    float ct = cos(substate.pose.z);
    substate.pose.x += dx*ct + dy*st;
    substate.pose.y -= -dx*st + dy*ct;  // image y is reversed from sub y
    substate.pose.z += dtheta;          // theta stored as 'z'
    
    // Write tf, pose to screen
    #ifdef DEBUG
    cout << "tf = "<< endl << " "  << odomdata.tf << endl << endl;
    cout << "pose = "<< endl << " " << substate.pose << endl << endl;
    #endif
    
    // Swap newframe and oldframe.
    cv::swap(odomdata.newimg,odomdata.oldimg);
    
    // Save feature information to "old" values
    newkeypoints.swap(oldkeypoints);
    cv::swap(newdescriptors,olddescriptors);
    
    // Check how much time it's taken to run this thread
    gettimeofday(&now, NULL);
    
    // Pause for the remainder of the time necessary to achieve desired rate
    int elapsedtime = (now.tv_sec - threadstart.tv_sec)*1000000 +
                      (now.tv_usec - threadstart.tv_usec);
//    auv_usleep(1000000/ODOM_RATE - elapsedtime);
    
  } // end while(...)
  
  return NULL;
} // end visualOdometry()


/*******************************************************************************
 * int initializeOdomDataLock(odom_data_t *_odomdata)
 *
 * Initialize locks for odom_data_t variable
 ******************************************************************************/
int initializeOdomDataLock(odom_data_t *_odomdata)
{
  if (pthread_mutex_init(&_odomdata->newimglock, NULL) != 0)
  {
    cout << "Unable to initialize newimglock" << endl;
    return 0;
  }
  if (pthread_mutex_init(&_odomdata->oldimglock, NULL) != 0)
  {
    cout << "Unable to initialize oldimglock" << endl;
    return 0;
  }
  return 1;
}