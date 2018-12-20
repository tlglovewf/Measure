#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
using namespace std;
using namespace cv;

typedef std::vector<KeyPoint>  KeyPointVector;
typedef std::vector<DMatch>    MatchVector;

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
    return Point2d
    (
     (p.x-K.at<double>(0,2))/K.at<double>(0,0),
     (p.y-K.at<double>(1,2))/K.at<double>(1,1)
     );
}

void triangulation( const KeyPointVector &keypoint_1, const KeyPointVector &keypoint_2, const MatchVector &matches,
                   const Mat &R, const Mat &t, vector<Point3d> &points)
{
    Mat T1 = (Mat_<double>(3,4) << 1,0,0,0,
                                   0,1,0,0,
                                   0,0,1,0,
                                    0,0,0,0);
    Mat T2 = (Mat_<double>(3,4) <<
              R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
              R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
              R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0));
    
    Mat K = (Mat_<double>(3,3) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02,
                                  0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02,
                                  0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00 );
 
    vector<Point2d> pts_1, pts_2;
    
    for(DMatch m : matches)
    {
        pts_1.push_back( pixel2cam( keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back( pixel2cam( keypoint_2[m.trainIdx].pt, K));
    }
    
    Mat pts_4d;
    triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    
    for( int i = 0; i < pts_4d .cols; ++i)
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0);
        Point3d p( x.at<float>(0,0),
                   x.at<float>(1,0),
                   x.at<float>(2,0));
        points.push_back(p);
        cout << "triangulate pos : " << p.x << " " << p.y << " " << p.z << endl;
    }
}

void pose_estimation_2d2d(const KeyPointVector &kypt1,const KeyPointVector &kypt2,
                          const MatchVector &matches, Mat &R, Mat &t)
{
//    Mat K = (Mat_<double>(3,3) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02 ,0.000000000000e+00,0.000000000000e+00 ,7.188560000000e+02 ,1.852157000000e+02, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00,0.000000000000e+00);
    
    vector<Point2f> points1;
    vector<Point2f> points2;
    
    for(int i = 0;i < (int)matches.size(); ++i)
    {
        points1.push_back( kypt1[matches[i].queryIdx].pt);
        points2.push_back( kypt2[matches[i].trainIdx].pt);
    }
    
    Mat fundamental_matrix;
    
    fundamental_matrix = findFundamentalMat( points1, points2, CV_FM_8POINT);
    
    Point2d principal_point( 607.1928,185.2157);
    int focal_length = 718.856;
    
    Mat essential_matrix = findEssentialMat(points1, points2, focal_length,principal_point,RANSAC);
    
    recoverPose( essential_matrix, points1, points2, R, t, focal_length,principal_point);
    
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
    
}


void find_matches( const Mat &img_1, const Mat &img_2,KeyPointVector &keypoints_1, KeyPointVector &keypoints_2,  MatchVector &matches)
{
    Mat descriptors_1, descriptors_2;
    Ptr<ORB> orb = ORB::create(500, 2.0f, 8 ,31, 0, 2, ORB::HARRIS_SCORE,31,20);
    
    orb->detect(img_1, keypoints_1);
    orb->detect(img_2, keypoints_2);
    
    orb->compute(img_1, keypoints_1, descriptors_1);
    orb->compute(img_2, keypoints_2, descriptors_2);
    
    vector<DMatch> tmpmatches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1, descriptors_2, tmpmatches);
    
    double min_dist = 10000, max_dist = 0;
    
    for(int i = 0; i < descriptors_1.rows;++i)
    {
        double dist = tmpmatches[i].distance;
        if(dist < min_dist)min_dist = dist;
        if(dist < max_dist)max_dist = dist;
    }
    
    cout << " mindist : " << min_dist << endl;
    cout << " maxdist : " << max_dist << endl;
    
    for(int i = 0; i < descriptors_1.rows;++i)
    {
        if( tmpmatches[i].distance <= max(2 * min_dist,30.0))
        {
            matches.push_back(tmpmatches[i]);
        }
    }
    
#if 0
    Mat img_match;
    Mat img_goodmatch;
    
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    
    imshow("all feautres", img_match);
    imshow("optimise features", img_goodmatch);
#endif
    
}

int main(void)
{
    Mat img_1 = imread("/Volumes/mac/Data/01/image_0/000000.png");
    Mat img_2 = imread("/Volumes/mac/Data/01/image_0/000001.png");
    
    KeyPointVector keypoints_1, keypoints_2;
    MatchVector matches;
    
    find_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    
    Mat R,t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
    
    vector<Point3d> pts;
    
    triangulation(keypoints_1, keypoints_2, matches, R, t, pts);
    
    waitKey(0);
    
   return 0;
}
