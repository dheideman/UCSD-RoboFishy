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
#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../Modules/TypeDefs.h"

// Multithreading
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#define DEBUG

#define OUTPUT_VIDEO_NAME "DemoVideo.avi"
#define FRAME_WIDTH       640
#define FRAME_HEIGHT      480
#define MIN_MINDIST       10
#define MAX_MATCHES       500
#define PI                3.141592653589793

#define MATCH_CIRCLE_R    5      // Match circle radius, px

using namespace cv;
using namespace std;

// Typedef of struct to store new/old images
typedef struct odom_data_t
{
  cv::Mat               newimg;   // The newer of the two images stored
  cv::Mat               oldimg;   // The older of the two images stored
  std::vector<Point2f>  newpts;   // The matched points in the new image
  std::vector<Point2f>  oldpts;   // The matched points in the new image
  cv::Mat               H;        // Transformation matrix between images
} odom_data_t;

// Global odometry image struct
odom_data_t odomdata;

// Transformation matrix
Mat H;

// Thread attributes for different priorities
pthread_attr_t tattrlow, tattrmed, tattrhigh;

// Sub Images
sub_images_t subimages;

// Global state variable
sub_state_t substate;

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

/////////////
// Threads //
/////////////

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
  
  // Save bright frame to "old image"
  subimages.brightframe.copyTo(odomdata.oldimg);
  
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
  
  while( substate.mode != RUNNING )
  {
    // Save bright frame to "old image"
    subimages.brightframe.copyTo(odomdata.newimg);
    
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
    odomdata.H = findHomography(odomdata.newpts, odomdata.oldpts, CV_RANSAC);
    
    // Transform reference points to determine motion
    perspectiveTransform(pretfpts, posttfpts, odomdata.H);
    
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
    
    // Write H, pose to screen
    #ifdef DEBUG
    cout << "H = "<< endl << " "  << odomdata.H << endl << endl;
    cout << "pose = "<< endl << " " << substate.pose << endl << endl;
    #endif
    
    // Swap newframe and oldframe.
    cv::swap(odomdata.newimg,odomdata.oldimg);
    
    // Save feature information to "old" values
    newkeypoints.swap(oldkeypoints);
    cv::swap(newdescriptors,olddescriptors);
    
  } // end while(...)
  
  return NULL;
} // end visualOdometry()

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
  
  // Initialize scheduling parameters, priorities
  sched_param param;
  int policy, maxpriority;
  
  // Initialize priorities
  pthread_attr_init(&tattrlow);
  pthread_attr_init(&tattrmed);
  pthread_attr_init(&tattrhigh);
  
  // Get max priority
  pthread_attr_getschedpolicy(&tattrlow, &policy);
  maxpriority = sched_get_priority_max(policy);
  
  // Extract scheduling parameter
  pthread_attr_getschedparam (&tattrlow, &param);
  
  // Set up low priority
  param.sched_priority = maxpriority/4;
  pthread_attr_setschedparam (&tattrlow, &param);
  
  // Set up medium priority
  param.sched_priority = maxpriority/2;
  pthread_attr_setschedparam (&tattrmed, &param);
  
  // Set up high priority
  param.sched_priority = maxpriority-1;
  pthread_attr_setschedparam (&tattrhigh, &param);
  
  // Thread handles
  pthread_t odometryThread;

  // Create threads using modified attributes
  pthread_create (&odometryThread, &tattrhigh, visualOdometry, NULL);
  
  

  // Destroy the thread attributes
  pthread_attr_destroy(&tattrlow);
  pthread_attr_destroy(&tattrmed);
  pthread_attr_destroy(&tattrhigh);
  
  sleep(1);
  
  // MAIN LOOP!
  while (waitKey(1) != 27)
  {
    // Get pictures
    cap.read(subimages.brightframe);
    
    // Create image?
    
    // Draw all those matches
// What do we do about newkeypoints and oldkeypoints?
//     drawMatches(odomdata.newimg, newkeypoints, odomdata.oldimg, oldkeypoints, goodmatches, matchesimg, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    
    // Transform corner points by H to get corresponding points on scene image
    perspectiveTransform(newcorners, oldcorners, odomdata.H);
    
    // Combine images onto image of matches (matchesimg)
    Mat left(matchesimg, Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT));
    odomdata.newimg.copyTo(left);
    Mat right(matchesimg, Rect(FRAME_WIDTH, 0, FRAME_WIDTH, FRAME_HEIGHT));
    odomdata.oldimg.copyTo(right);
    
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