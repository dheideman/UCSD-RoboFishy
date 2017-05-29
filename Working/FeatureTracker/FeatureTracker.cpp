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

#define OUTPUT_VIDEO_NAME "DemoVideo.avi"
#define FRAME_WIDTH       1280
#define FRAME_HEIGHT      720
#define MIN_MINDIST       10
#define MAX_N_MATCHES     50
#define PI                3.141592653589793

using namespace cv;
using namespace std;

// Points to transform to calculate motion
// [0] = center point
// [1] = top-center point
vector<Point2f> pretfpts(2);    // Pre TF Pts
vector<Point2f> posttfpts(2);   // Post TF Pts
vector<Point2f> deltatfpts(2);  // Delta TF Pts

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
  
    // Set video capture settings:
/*
  // Get Codec Type- Int form
  int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
  
  // Acquire input size 
  Size framesize = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH)*2,
                        (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  
  // Get FPS
  float fps = 5;//cap.get(CV_CAP_PROP_FPS);
  
  // Initialize video writer
  VideoWriter outputvideo;
  outputvideo.open(OUTPUT_VIDEO_NAME, ex, fps, framesize, true);
//*/
  
  // Initialize transformpoints
  pretfpts[0] = cvPoint(FRAME_WIDTH/2, FRAME_HEIGHT/2); // center
  pretfpts[1] = cvPoint(FRAME_WIDTH/2, 0); // center-top
  
  // Initialize frames
  Mat newimg, oldimg;
  
  // Clear buffer
  for(int i=0; i<5; i++)
  {
    cap.read(oldimg);
  }
  
  
  // MAIN LOOP!
  while (waitKey(1) != 27)
  {
    // Get pictures
    cap.read(newimg);
    /* Step 1: Detect the keypoints using Fast Detector */
    
    // Create detector
//     Ptr<FeatureDetector> detector = ORB::create();
    Ptr<FeatureDetector> detector = FastFeatureDetector::create();

    // Create vector of KeyPoints for each image 
    vector<KeyPoint> newkeypoints, oldkeypoints;

    // Detect features (KeyPoints) in each image
    detector->detect(newimg, newkeypoints);
    detector->detect(oldimg, oldkeypoints);
    
    // Print out how many KeyPoints were detected in each image (DEBUG)
//     cout << "# KeyPoints, object: " << newkeypoints.size() << endl;
//     cout << "# KeyPoints, scene:  " << oldkeypoints.size() << endl;
    
    
    /* Step 2: Calculate descriptors (feature vectors) */
    
    // Create extractor (will characterize each feature)
    Ptr<DescriptorExtractor> extractor = ORB::create();
    
    // Create 2D matrix for descriptors for each image
    // Each row corresponds to a feature
    // 32 columns describe each feature
    Mat newdescriptors, olddescriptors;
        
    // Characterize each feature, store characteristics in Mats
    extractor->compute(newimg, newkeypoints, newdescriptors);
    extractor->compute(oldimg, oldkeypoints, olddescriptors);
    
    // Make sure we have some descriptors, otherwise return error.
    if ( newdescriptors.empty() )
    {
      cerr << "Error: newdescriptors is empty" << endl;
      return -1;
    }
    if ( olddescriptors.empty() )
    {
      cerr << "Error: olddescriptors is empty" << endl;
      return -1;
    }
    
    // Convert descriptors to CV_32F type for FLANN
    if(newdescriptors.type()!=CV_32F)
    {
        newdescriptors.convertTo(newdescriptors, CV_32F);
    }

    if(olddescriptors.type()!=CV_32F)
    {
        olddescriptors.convertTo(olddescriptors, CV_32F);
    }

    /* Step 3: Matching descriptor vectors using FLANN matcher */
    
    // Create matcher
    // FLANN
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
    // Brute Force:
//     Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    
    // Create vector of matches
    vector< DMatch > matches;
    
    // Calculate matches between two images
    matcher->match(newdescriptors, olddescriptors, matches);

    // Calculate max and min distances between matched points
    double avgdist = 0;
    double mindist = FRAME_WIDTH;
    for (int i = 0; i < newdescriptors.rows; i++)
    {
      double dist = matches[i].distance;
      avgdist += dist;
      if (dist < mindist) mindist = dist;
    }
    avgdist = avgdist/newdescriptors.rows;
    if(mindist < MIN_MINDIST) mindist = MIN_MINDIST;
    
    // Calculate standard deviation
    double stddev = 0;
    double avgdist2 = pow(avgdist,2);
    for (int i = 0; i < newdescriptors.rows; i++)
    {
      stddev += (pow(matches[i].distance,2) - avgdist2);
    }
    stddev = sqrt(stddev/newdescriptors.rows);
    cout << "Avg Distance = " << avgdist << endl;
    cout << "Standard Dev = " << stddev << endl;

    /* Draw only "good" matches (i.e. whose distance is less than 3*min_dist) */
    // Sort matches by distance: shortest to longest
    sort(matches.begin(), matches.end(), compareDMatch);
    
    // Create vector of "good" matches
    vector< DMatch > goodmatches;
    
    // Iterate through matches?
//     for (int i = 0; i < newdescriptors.rows; i++)
//     for (int i = 0; i < matches.size(); i++)
//     {
//       if (matches[i].distance <= 3*mindist + 0.1*FRAME_HEIGHT)
//       {
//         goodmatches.push_back(matches[i]);
//       }
//     }
    
    // Write number of "good" matches to screen (DEBUG)
    cout << "matches.size() = " << matches.size() << endl;
    cout << "goodmatches.size() = " << goodmatches.size() << endl;
    cout << "goodmatches:" << endl;
    
    int nmatches = matches.size();
    int ngoodmatches = (nmatches/2 > MAX_N_MATCHES ? MAX_N_MATCHES : nmatches/2);
    
    for(int i=0; i<ngoodmatches; i++)
    {
      goodmatches.push_back(matches[i]);
      cout << "\t" << goodmatches[i].distance << endl;
    }
    
    if ( goodmatches.size() == 0 )
    {
      cerr << "Error: no good matches detected" << endl;
      return -1;
    }
    
    // Create matrix to store final output image
    Mat matchesimg;
    
    // Draw all those matches
    drawMatches(newimg, newkeypoints, oldimg, oldkeypoints, goodmatches, matchesimg, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    /* Localize the object */
    
    // Create vectors of all the coordinates of the "good" matches
    vector<Point2f> newpoints;
    vector<Point2f> oldpoints;

    for (int i = 0; i < goodmatches.size(); i++)
    {
        // Get the keypoints from the good matches
        newpoints.push_back(newkeypoints[goodmatches[i].queryIdx].pt);
        oldpoints.push_back(oldkeypoints[goodmatches[i].trainIdx].pt);
    }
    
    // Write sizes of the localization vectors
//     cout << "newpoints.size() = " << newpoints.size() << endl;
//     cout << "oldpoints.size() = " << oldpoints.size() << endl;
     
    // Find perspective transformation between two planes.
    Mat H = findHomography(newpoints, oldpoints, CV_RANSAC);

    // Create point arrays for image corners
    vector<Point2f> newcorners(4);
    vector<Point2f> oldcorners(4);
    
    // Create array of corner points of object image
    newcorners[0] = cvPoint(0, 0);
    newcorners[1] = cvPoint(newimg.cols, 0);
    newcorners[2] = cvPoint(newimg.cols, newimg.rows);
    newcorners[3] = cvPoint(0, newimg.rows);
    
    // Transform corner points by H to get corresponding points on scene image
    perspectiveTransform(newcorners, oldcorners, H);

    // Draw box around detected object in scene image
    line(matchesimg, oldcorners[0] + Point2f(newimg.cols, 0), oldcorners[1] + Point2f(newimg.cols, 0), Scalar(0, 255, 0), 4);
    line(matchesimg, oldcorners[1] + Point2f(newimg.cols, 0), oldcorners[2] + Point2f(newimg.cols, 0), Scalar(0, 255, 0), 4);
    line(matchesimg, oldcorners[2] + Point2f(newimg.cols, 0), oldcorners[3] + Point2f(newimg.cols, 0), Scalar(0, 255, 0), 4);
    line(matchesimg, oldcorners[3] + Point2f(newimg.cols, 0), oldcorners[0] + Point2f(newimg.cols, 0), Scalar(0, 255, 0), 4);  
    
    // Write H to screen
    cout << "H = "<< endl << " "  << H << endl << endl;
    
    // Transform reference points to determine motion
    perspectiveTransform(pretfpts, posttfpts, H);
    
    // Calculate pure dx, dy of points (transformed - initial position
    deltatfpts[0] = posttfpts[0] - pretfpts[0]; // center
    deltatfpts[1] = posttfpts[1] - pretfpts[0]; // top-center
    
    // Calculate dtheta using top-center motion
    float dtheta = atan2(-1*deltatfpts[1].x, -1*deltatfpts[1].y);
    cout << dtheta*180/PI << endl;
    
    // Calculate new pose    
    float dx = deltatfpts[0].x;
    float dy = deltatfpts[0].y;
    float st = sin(substate.pose.z);
    float ct = cos(substate.pose.z);
    substate.pose.x += st*dx + ct*dy;
    substate.pose.y += ct*dx - st*dy;
    substate.pose.z += dtheta;
    
    // Write pose to screen
    cout << "pose = "<< endl << " "  << substate.pose << endl << endl;
    
    // Show detected matches
    imshow("Good Matches & Object detection", matchesimg);
    
    // Write frame to video
//     outputvideo.write(matchesimg);
    
    // Copy newframe to oldframe.
    newimg.copyTo(oldimg);
  }
  return 0;
}