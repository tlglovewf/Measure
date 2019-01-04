#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/video/tracking.hpp>

#include "MathHelper.h"
#include <iomanip>
using namespace std;
using namespace cv;

typedef std::vector<KeyPoint>  KeyPointVector;
typedef std::vector<DMatch>    MatchVector;



/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
                          Matx34d P,       //camera 1 matrix
                          Point3d u1,      //homogenous image point in 2nd camera
                          Matx34d P1       //camera 2 matrix
)
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
    Mat B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3)),
             -(u.y*P(2,3)  -P(1,3)),
             -(u1.x*P1(2,3)    -P1(0,3)),
             -(u1.y*P1(2,3)    -P1(1,3)));
    
    Mat X;
    solve(A,B,X,DECOMP_SVD);
    
    return X;
}


Mat_<double> IterativeLinearLSTriangulation(Point3d u,    //homogenous image point (u,v,1)
                                            Matx34d P,          //camera 1 matrix
                                            Point3d u1,         //homogenous image point in 2nd camera
                                            Matx34d P1          //camera 2 matrix
) {
    double wi = 1, wi1 = 1;
    Mat_<double> X(4,1);
    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
        
        //recalculate weights
        double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
        double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
#define EPSILON 1.0e-6
        //breaking point
        if(fabsf(wi - p2x) <= DBL_EPSILON && fabsf(wi1 - p2x1) <= DBL_EPSILON)
        {
            break;
        }
        wi = p2x;
        wi1 = p2x1;
        
        //reweight equations and solve
        Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
        
        solve(A,B,X_,DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    }
    return X;
}








//计算距离  m
double triangulation( const Point2d &pt1,const Point2d &pt2,double scale, const Mat &R, const Mat &t,const Mat &K, Point3d &outpt, double realdistance = 0)
{
    Mat T1 = (Mat_<double>(3,4) << 1,0,0,0,
                                   0,1,0,0,
                                   0,0,1,0);
    Mat T2 = (Mat_<double>(3,4) <<
              R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0) ,
              R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0) ,
              R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0) );
   
    Mat pt_4d;

#if 0
    vector<Point2d> pts_1, pts_2;

    pts_1.push_back(GeoMath::pixel2cam(pt1, K));
    pts_2.push_back(GeoMath::pixel2cam(pt2, K));
    
    triangulatePoints(T1, T2, pts_1, pts_2, pt_4d);
    
#else
   Point2d dp1 = GeoMath::pixel2cam(pt1, K);
   Point2d dp2 = GeoMath::pixel2cam(pt2, K);
   pt_4d = IterativeLinearLSTriangulation(Point3d(dp1.x,dp2.y,0), T1, Point3d(dp2.x,dp2.y,0), T2);
#endif
    Mat x = pt_4d.col(0);
    
    x = x/x.at<double>(3,0);
    outpt = Point3d(x.at<double>(0,0),x.at<double>(1,0),x.at<double>(2,0));
    
    double d = scale / sqrt(t.at<double>(0,0) * t.at<double>(0,0)+
                            t.at<double>(1,0) * t.at<double>(1,0)+
                            t.at<double>(2,0) * t.at<double>(2,0));
    
    double len = sqrt(outpt.x * outpt.x + outpt.y * outpt.y + outpt.z * outpt.z) * d;
    
    outpt = outpt * d;
    
    cout << "ouput pos :" << outpt.x  << " " << outpt.y << " " << outpt.z << endl;
    cout << "camera real distance : " << (len - realdistance) << endl;
    
    return len ;
}

void featureDetection(Mat img_1, vector<Point2f>& points1)    {   //uses FAST as of now, modify parameters as necessary
    vector<KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)    {
    
    //this function automatically gets rid of points for which tracking fails
    
    vector<float> err;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
    
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
    {  Point2f pt = points2.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))    {
            if((pt.x<0)||(pt.y<0))    {
                status.at(i) = 0;
            }
            points1.erase (points1.begin() + (i - indexCorrection));
            points2.erase (points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
        
    }
    
}

void pose_estimation_2d2d(const Mat &img1, const Mat &img2, Mat &R, Mat &t,const Point2d &principal_point, double focal_length)
{
    Mat img_out1,img_out2;
    cvtColor(img1, img_out1, CV_BGR2GRAY);
    cvtColor(img2, img_out2, CV_BGR2GRAY);
    vector<uchar> status;
    vector<Point2f> points1;
    vector<Point2f> points2;
    featureDetection(img_out1,points1);
    featureTracking(img_out1, img_out2, points1, points2, status);
    
    Mat fundamental_matrix;
    
    fundamental_matrix = findFundamentalMat( points1, points2, CV_FM_8POINT);
    
    Mat essential_matrix = findEssentialMat(points1, points2, focal_length,principal_point,RANSAC);
    
    recoverPose( essential_matrix, points1, points2, R, t, focal_length,principal_point);
}

//恢复姿态
void pose_estimation_2d2d(const KeyPointVector &kypt1,const KeyPointVector &kypt2,
                          const MatchVector &matches, Mat &R, Mat &t,const Point2d &principal_point, double focal_length)
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
    
    Mat essential_matrix = findEssentialMat(points1, points2, focal_length,principal_point,RANSAC);
    
    recoverPose( essential_matrix, points1, points2, R, t, focal_length,principal_point);
}

//特征点匹配
void find_matches( const Mat &img_1, const Mat &img_2,KeyPointVector &keypoints_1, KeyPointVector &keypoints_2,  MatchVector &matches)
{
    Mat descriptors_1, descriptors_2;
    Ptr<ORB> orb = ORB::create(500, 2.0f, 8 ,31, 0, 2, ORB::HARRIS_SCORE,31,20);
    
    orb->detect(img_1, keypoints_1);
    orb->detect(img_2, keypoints_2);
    
    orb->compute(img_1, keypoints_1, descriptors_1);
    orb->compute(img_2, keypoints_2, descriptors_2);
    
    
    MatchVector tmpmatches;
#if 1
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
#else
  
    
#endif
    
    cout << "matches count is : " << matches.size() << endl;
    
#if 1
    //draw match image
    Mat img_match;
    Mat img_goodmatch;
    
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    
    imwrite("/Volumes/mac/Data/measure/match.png", img_match);
#endif
    
}


cv::Point2d  ComputeGPSFromCamPos(const cv::Point3d &campos, const cv::Point3d &preGps, const cv::Point3d &curGps,const cv::Point2d &target)
{
    cv::Point3d pt1 = GeoMath::ComputeXYZFromGPS(preGps.x, preGps.y);
    cv::Point3d pt2 = GeoMath::ComputeXYZFromGPS(curGps.x, curGps.y);
    
    cv::Point3d dir = pt2 - pt1;
    
    GeoMath::normalize(dir);
    
#if 0
    //根据行驶方向以及相对相机z轴夹角
    double θ = atan(campos.x / campos.z);
    
    cv::Point3d cdir(dir.x*cos(θ) + dir.y * sin(θ), -dir.y*cos(θ) + dir.x * sin(θ),0);
    GeoMath::normalize(cdir);
    double len = GeoMath::getLength(campos);
    pt2 =  pt2 + len * cdir;
    
    cv::Point2d gg= GeoMath::ComputeGPSFromXYZ(pt2);
    std::cout<< std::setprecision(10) << gg.x << "," << gg.y << " absolute pos :" <<
    GeoMath::ComputeDistance(gg.x,gg.y,target.x,target.y) << std::endl;
    
#else
    cv::Point3d zAixs = pt2 - pt1;  //行驶方向(相机拍摄方向）
    GeoMath::normalize(zAixs);      //单位化
    cv::Point3d yAixs = pt2;        //up方向
    GeoMath::normalize(yAixs);      //单位化
    Mat R = GeoMath::ComputeWorldTransMatrix(zAixs, yAixs, pt2);//获取世界变换矩阵
    
    Mat pos = (Mat_<double>(4,1) << campos.x,campos.y,campos.z,1) ;
    
    Mat rst = R * pos;
    
    cv::Point3d rstpt(rst.at<double>(0,0)/rst.at<double>(3,0),
                      rst.at<double>(1,0)/rst.at<double>(3,0),
                      rst.at<double>(2,0)/rst.at<double>(3,0));
    
    cv::Point2d  gp = GeoMath::ComputeGPSFromXYZ(rstpt);
    
    std::cout << std::setprecision(20)  << "prerpt :"<< preGps.y << ","<< preGps.x << endl<<
                                           "currpt :"<< curGps.y << ","<< curGps.x << endl<<
                                           "target :"<< target.y << ","<< target.x << endl<<
                                           "output :" << gp.y << "," << gp.x << endl << "absolute distance : "
    << GeoMath::ComputeDistance(gp.x, gp.y, target.x, target.y) <<  std::endl;
    
    return gp;
#endif
}


int main(void)
{
    
#if 1
    Mat img_1 = imread("/Volumes/mac/Data/measure/4014.jpg");
    Mat img_2 = imread("/Volumes/mac/Data/measure/4015.jpg");
    
    //相机内参
    Mat K = (Mat_<double>(3,3) <<  1.8144486313396042e+03,0  ,2.0521093136805948e+03,
             0, 1.8144486313396042e+03, 1.0898609600712157e+03,
             0, 0, 1);
    
    Point2d  principal_point( 2.0521093136805948e+03,1.0898609600712157e+03);//光心坐标
    int      focal_length = 1.8144486313396042e+03;                          //焦距
    
    Point2d pt1(2735  , 850);
    Point2d pt2(2879.9, 811.267);
    
    cv::Point3d preGps(114.40327060991, 30.60129500574,0);
    cv::Point3d curGps(114.40331230623, 30.60129672252,0);
    
    double scale = GeoMath::ComputeDistance(preGps.x,preGps.y,
                                            curGps.x,curGps.y);
    
    double realdistance = 19.5855;
    
    cv::Point2d target(114.40350395, 30.60123760);
#elif 0
    Mat img_1 = imread("/Volumes/mac/Data/measure/1_1.jpg");
    Mat img_2 = imread("/Volumes/mac/Data/measure/1_2.jpg");
    
    //相机内参
    Mat K = (Mat_<double>(3,3) <<  2.3695365586649123e+3,0  ,2.0443736115794320e+3,
             0, 2.3695365586649123e+3, 1.0750972331437883e+3,
             0, 0, 1);
    
    Point2d  principal_point( 2.0443736115794320e+3,1.0750972331437883e+3);//光心坐标
    int      focal_length = 2.3695365586649123e+3;                         //焦距
    
    Point2d pt1(2979.1,711.3);
    Point2d pt2(3080.6,699.864);
    
    cv::Point3d preGps(114.47140275, 30.45029056,0); //19.0043
    cv::Point3d curGps(114.47144438, 30.45029059,0); //18.9148
    
    cv::Point2d target(114.47181491, 30.45015864);
    
    double scale = GeoMath::ComputeDistance(preGps.x,preGps.y,
                                            curGps.x,curGps.y);
    
    double realdistance = 38.5330;
#else
    Mat img_1 = imread("/Volumes/mac/Data/measure/2_1.jpg");
    Mat img_2 = imread("/Volumes/mac/Data/measure/2_2.jpg");
    
    //相机内参
    Mat K = (Mat_<double>(3,3) <<  2.3695365586649123e+3,0  ,2.0443736115794320e+3,
             0, 2.3695365586649123e+3, 1.0750972331437883e+3,
             0, 0, 1);
    
    Point2d  principal_point( 2.0443736115794320e+3,1.0750972331437883e+3);//光心坐标
    int      focal_length = 2.3695365586649123e+3;                         //焦距
    
    Point2d pt1(2975.7,983.8);
    Point2d pt2(3274.0,998.0);

    cv::Point3d preGps(114.47152763, 30.45029023,0); //18.7480
    cv::Point3d curGps(114.47156924, 30.45029004,0); //18.6678
    
    cv::Point2d target(114.47170243, 30.45023748);
    
    double scale = GeoMath::ComputeDistance(preGps.x,preGps.y,
                                            curGps.x,curGps.y);
    
    double realdistance = 17.8044;
    
    
#endif
//    cv::cvtColor(img_1,img_1,CV_RGB2GRAY);//灰度
//    cv::cvtColor(img_2,img_2,CV_RGB2GRAY);
    
    KeyPointVector keypoints_1, keypoints_2;
    MatchVector matches;
    
  
    Mat R,t;
    
#if 1
    find_matches(img_1, img_2, keypoints_1, keypoints_2, matches);           //特征匹配
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t,principal_point,focal_length);
#else
    pose_estimation_2d2d(img_1, img_2, R, t, principal_point, focal_length); //光流
#endif
    vector<Point3d> pts;
  
    Point3d outpt;
    triangulation(pt1, pt2,scale, R, t,K, outpt,realdistance);
    
    ComputeGPSFromCamPos(outpt, preGps, curGps,target);
    
    //waitKey(0);
    
   return 0;
}
