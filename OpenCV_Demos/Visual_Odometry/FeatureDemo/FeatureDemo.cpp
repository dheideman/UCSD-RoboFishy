/******************************************************************************
 * FeatureDemo.cpp
 * 
 * Trying some FLANN stuff out for feature tracking
 * 
 * Person on the Internet:
 * http://stackoverflow.com/questions/31835114/feature-detection-with-patent-free-descriptors
 *
 * FLANN: 
 * http://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html?highlight=flannbasedmatcher#flannbasedmatcher
 ******************************************************************************/
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//////////
// Main //
//////////
int main(int argc, char** argv)
{
    if( argc != 3 )
    {
      cout << " Usage: ./FeatureDemo <img1> <img2>" << endl;
      return -1;
    }
    
    Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
    Mat img_scene = imread( argv[2], IMREAD_GRAYSCALE );

    if( !img_object.data || !img_scene.data )
    {
      cout<< " --(!) Error reading images " << endl;
      return -1;
    }

    /* Step 1: Detect the keypoints using Fast Detector */
    
    // Create detector
//     Ptr<FeatureDetector> detector = ORB::create();
    Ptr<FeatureDetector> detector = FastFeatureDetector::create();

    // Create vector of KeyPoints for each image 
    vector<KeyPoint> keypoints_object, keypoints_scene;

    // Detect features (KeyPoints) in each image
    detector->detect(img_object, keypoints_object);
    detector->detect(img_scene, keypoints_scene);
    
    // Print out how many KeyPoints were detected in each image (DEBUG)
    cout << "# KeyPoints, object: " << keypoints_object.size() << endl;
    cout << "# KeyPoints, scene:  " << keypoints_scene.size() << endl;
    
    
    /* Step 2: Calculate descriptors (feature vectors) */
    
    // Create extractor (will characterize each feature)
    Ptr<DescriptorExtractor> extractor = ORB::create();
//     Ptr<DescriptorExtractor> extractor = KAZE::create();
    
    cout << "Created Extractor" << endl;

    // Create 2D matrix for descriptors for each image
    // Each row corresponds to a feature
    // 32 columns describe each feature
    Mat descriptors_object, descriptors_scene;
    
    cout << "Created descriptor matrices" << endl;
    
    // Characterize each feature, store characteristics in Mats
    extractor->compute(img_object, keypoints_object, descriptors_object);
    extractor->compute(img_scene, keypoints_scene, descriptors_scene);
    
    // Print number of rows, columns for each image (DEBUG)
    cout << "# Rows, object: " << descriptors_object.rows << endl;
    cout << "# Cols, object: " << descriptors_object.cols << endl;
    cout << "# Rows, scene:  " << descriptors_scene.rows << endl;
    cout << "# Cols, scene:  " << descriptors_scene.cols << endl;
    
    // Make sure we have some descriptors, otherwise return error.
    if ( descriptors_object.empty() )
    {
      cerr << "Error: descriptors_object is empty" << endl;
      return -1;
    }
    if ( descriptors_scene.empty() )
    {
      cerr << "Error: descriptors_scene is empty" << endl;
      return -1;
    }

    /* Step 3: Matching descriptor vectors using FLANN matcher */
    
    // Create matcher
//     Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
    // Brute Force:
//     Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    
    // Create vector of matches
    vector< DMatch > matches;
    
    // Calculate matches between two images
    matcher->match(descriptors_object, descriptors_scene, matches);
    
    // Write number of matches to screen (DEBUG)
    cout << "matches.size() = " << matches.size() << endl;

    // Calculate max and min distances between matched points
    double max_dist = 0; double min_dist = 200; double avg_dist = 0;
    for (int i = 0; i < descriptors_object.rows; i++)
    {
        double dist = matches[i].distance;
        avg_dist += dist;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    avg_dist = avg_dist/descriptors_object.rows;
    
    // Write min and max distances to screen
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);
    printf("-- Avg dist : %f \n", avg_dist);

    /* Draw only "good" matches (i.e. whose distance is less than 3*min_dist) */
    
    // Create vector of "good" matches
    vector< DMatch > good_matches;
    
    // Iterate through matches?
//     for (int i = 0; i < descriptors_object.rows; i++)
    for (int i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance <= /*avg_dist/2*/ 2 * (min_dist+1))
        {
            good_matches.push_back(matches[i]);
        }
    }
    
    // Write number of "good" matches to screen (DEBUG)
    cout << "good_matches.size() = " << good_matches.size() << endl;
    
    if ( good_matches.size() == 0 )
    {
      cerr << "Error: no good matches detected" << endl;
      return -1;
    }
    
    // Create matrix to store final output image
    Mat img_matches;
    
    // Draw all those matches
    drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    /* Localize the object */
    
    // Create vectors of all the coordinates of the "good" matches
    vector<Point2f> obj;
    vector<Point2f> scene;

    for (int i = 0; i < good_matches.size(); i++)
    {
        // Get the keypoints from the good matches
        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }
    
    // Write sizes of the localization vectors
    cout << "obj.size() = " << obj.size() << endl;
    cout << "scene.size() = " << scene.size() << endl;
     
    // Find perspective transformation between two planes.
    Mat H = findHomography(obj, scene, CV_RANSAC);

    // Create point arrays for image corners
    vector<Point2f> obj_corners(4);
    vector<Point2f> scene_corners(4);
    
    // Create array of corner points of object image
    obj_corners[0] = cvPoint(0, 0);
    obj_corners[1] = cvPoint(img_object.cols, 0);
    obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
    obj_corners[3] = cvPoint(0, img_object.rows);
    
    // Transform corner points by H to get corresponding points on scene image
    perspectiveTransform(obj_corners, scene_corners, H);

    // Draw box around detected object in scene image
    line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);  
    
    // Write H to screen
    cout << "H = "<< endl << " "  << H << endl << endl;
    
    // Show detected matches
    imshow("Good Matches & Object detection", img_matches);
    
//     imwrite("./SendaiTasselTest1b.jpg",img_matches);

    waitKey(0);
    return 0;
}