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
#define EARTH_RADIUS 6378137.0
const double Pi = 3.1415926;
double rad(double degree)
{
    return degree * Pi / 180.0f;
}
//计算距离  m
double computeDistance(double lng1, double lat1, double lng2, double lat2)
{
    double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = rad(lng1) - rad(lng2);
    
    double s = 2 * asin(sqrt(pow(sin(a/2),2) +
                             cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS;
    s = round(s * 10000) / 10000;
    return s;
}
double triangulation( const Point2d &pt1,const Point2d &pt2, const Mat &R, const Mat &t, Point3d &outpt)
{
    Mat T1 = (Mat_<double>(3,4) << 1,0,0,0,
                                   0,1,0,0,
                                   0,0,1,0);
    Mat T2 = (Mat_<double>(3,4) <<
              R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0) ,
              R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0) ,
              R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0) );
    
    Mat K = (Mat_<double>(3,3) <<  1.8144486313396042e+03,0  ,2.0521093136805948e+03,
                                   0, 1.8144486313396042e+03, 1.0898609600712157e+03,
                                   0, 0, 1);
   
#if 0
    Mat R1,R2,P1,P2,Q;
    
    Mat D = (Mat_<double>(14,1) << 2.7143309169665835e+00 ,7.6144381712092502e-01 ,4.2032650006901159e-04,
             2.0658547225350938e-05 ,-3.9272644112434946e-01,
             2.8270485246735952e+00 ,1.0376228716992399e+00,
             -5.2110054743233547e-01 ,0 , 0 , 0 , 0 , 0 , 0);
    
    stereoRectify(K, D, K, D, Size(4096,2168), R, t, R1, R2, P1, P2, Q,0);
    
    cout << "P1 : " << P1 << endl;
    cout << "P2 : " << P2 << endl;
#endif
    Mat pt_4d;
    //Point2d pt1(30.60129500574, 114.40327060991);//, 22.064 };
    //    Point2d pt2(30.60129672252, 114.40331230623);//, 22.095 };
    vector<Point2d> pts_1, pts_2;
    
    pts_1.push_back(pixel2cam(pt1, K));
    pts_2.push_back(pixel2cam(pt2, K));
    
    triangulatePoints(T1, T2, pts_1, pts_2, pt_4d);
    
    Mat x = pt_4d.col(0);
    
    x = x/x.at<double>(3,0);
    outpt = Point3d(x.at<double>(0,0),x.at<double>(1,0),x.at<double>(2,0));
    
    
    double scale = computeDistance(114.40327060991,30.60129500574,
                                   114.40331230623,30.60129672252);
    double d = scale / sqrt(t.at<double>(0,0) * t.at<double>(0,0)+
                            t.at<double>(1,0) * t.at<double>(1,0)+
                            t.at<double>(2,0) * t.at<double>(2,0));
    
    double len = sqrt(outpt.x * outpt.x + outpt.y * outpt.y + outpt.z * outpt.z) * d;
    cout << "ouput pos :" << outpt.x  << " " << outpt.y << " " << outpt.z << " " << (len - 19.5855) << endl;
    
    return len ;
}

void pose_estimation_2d2d(const KeyPointVector &kypt1,const KeyPointVector &kypt2,
                          const MatchVector &matches, Mat &R, Mat &t)
{
    vector<Point2f> points1;
    vector<Point2f> points2;
    
    for(int i = 0;i < (int)matches.size(); ++i)
    {
        points1.push_back( kypt1[matches[i].queryIdx].pt);
        points2.push_back( kypt2[matches[i].trainIdx].pt);
    }
    
    Mat fundamental_matrix;
    
    fundamental_matrix = findFundamentalMat( points1, points2, CV_FM_8POINT);
    
    Point2d principal_point( 2.0521093136805948e+03,1.0898609600712157e+03);//光心坐标
    int focal_length = 1.8144486313396042e+03;                              //焦距
    
    Mat essential_matrix = findEssentialMat(points1, points2, focal_length,principal_point,RANSAC);
    
    recoverPose( essential_matrix, points1, points2, R, t, focal_length,principal_point);
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
        if(dist > max_dist)max_dist = dist;
    }
    
    cout << " mindist : " << min_dist << endl;
    cout << " maxdist : " << max_dist << endl;
    
    for(int i = 0; i < tmpmatches.size();++i)
    {
        const int sz = 10;
        Rect2f rect(0,0,sz,sz);
        if( (keypoints_1[tmpmatches[i].queryIdx].pt - keypoints_2[tmpmatches[i].trainIdx].pt).inside(rect))
            continue;
//        Rect2f  rect(0,1600,4096,600);
//        if(keypoints_1[tmpmatches[i].queryIdx].pt.inside(rect) )
//            continue;
//
        if( tmpmatches[i].distance <= max(2 * min_dist,30.0) )
        {
            matches.push_back(tmpmatches[i]);
        }
    }
    
    cout << "matches count is : " << matches.size() << endl;
    
#if 1
    Mat img_match;
    Mat img_goodmatch;
    
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
//    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    
//    imshow("all feautres", img_match);
    imwrite("/Volumes/mac/Data/measure/match.png", img_match);
//    imshow("optimise features", img_goodmatch);
#endif
    
}


int main(void)
{   
    Mat img_1 = imread("/Volumes/mac/Data/measure/4014.jpg");
    Mat img_2 = imread("/Volumes/mac/Data/measure/4015.jpg");
    
    KeyPointVector keypoints_1, keypoints_2;
    MatchVector matches;
    
    find_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    
    Mat R,t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
    
    vector<Point3d> pts;
    
    Point2d pt1(2735, 850);//, 22.064 };
    Point2d pt2(2880, 810);//, 22.095 };
    
    Point3d outpt;
    triangulation(pt1, pt2, R, t, outpt);
    
    waitKey(0);
    
   return 0;
}
