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

#define OUTPUT_VIDEO_NAME "DemoVideo.avi"

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
  
  // Initialize coordinates
  Mat pose = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  double x = 0;
  double y = 0;
  
  // Initialize frames
  Mat newimg, oldimg;
  
  for(int i=0; i<10; i++)
  {
    cap.read(oldimg);
  }
  
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
//     Ptr<DescriptorExtractor> extractor = KAZE::create();
    
    // Create 2D matrix for descriptors for each image
    // Each row corresponds to a feature
    // 32 columns describe each feature
    Mat newdescriptors, olddescriptors;
        
    // Characterize each feature, store characteristics in Mats
    extractor->compute(newimg, newkeypoints, newdescriptors);
    extractor->compute(oldimg, oldkeypoints, olddescriptors);
    
    // Print number of rows, columns for each image (DEBUG)
//     cout << "# Rows, object: " << newdescriptors.rows << endl;
//     cout << "# Cols, object: " << newdescriptors.cols << endl;
//     cout << "# Rows, scene:  " << olddescriptors.rows << endl;
//     cout << "# Cols, scene:  " << olddescriptors.cols << endl;
    
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
    
    // Write number of matches to screen (DEBUG)
//     cout << "matches.size() = " << matches.size() << endl;

    // Calculate max and min distances between matched points
    double max_dist = 0; double min_dist = 200; double avg_dist = 0;
    for (int i = 0; i < newdescriptors.rows; i++)
    {
        double dist = matches[i].distance;
        avg_dist += dist;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    avg_dist = avg_dist/newdescriptors.rows;
    
    // Write min and max distances to screen
//     printf("-- Max dist : %f \n", max_dist);
//     printf("-- Min dist : %f \n", min_dist);
//     printf("-- Avg dist : %f \n", avg_dist);

    /* Draw only "good" matches (i.e. whose distance is less than 3*min_dist) */
    
    // Create vector of "good" matches
    vector< DMatch > goodmatches;
    
    // Iterate through matches?
//     for (int i = 0; i < newdescriptors.rows; i++)
    for (int i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance <= /*avg_dist/2*/ 10 * (min_dist+1))
        {
            goodmatches.push_back(matches[i]);
        }
    }
    
    // Write number of "good" matches to screen (DEBUG)
    cout << "goodmatches.size() = " << goodmatches.size() << endl;
    
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
    
    // Update position estimate
    x -= H.at<double>(0, 2);
    y -= H.at<double>(1, 2);
    
    // Calculate new pose
    pose = H*pose;
    
    // Write pose to screen
    cout << "pose = "<< endl << " "  << pose << endl << endl;
    
//     cout << "X: " << x << endl;
//     cout << "Y: " << y << endl;
//     cout << "X: " << H.at<double>(0, 2) << endl;
//     cout << "Y: " << H.at<double>(1, 2) << endl;
    
    // Show detected matches
    imshow("Good Matches & Object detection", matchesimg);
    
    // Write frame to video
//     outputvideo.write(matchesimg);
    
    // Copy newframe to oldframe.
    newimg.copyTo(oldimg);
//     imwrite("./SendaiTasselTest1b.jpg",matchesimg);
//     waitKey(0);
  }
  return 0;
}