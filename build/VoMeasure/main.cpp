/*

The MIT License

Copyright (c) 2015 Avi Singh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "vo_features.h"
#include "../../MathHelper.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

#define IMAGEPATH "/Volumes/mac/Data/12"
#define REALPOSE  "/Volumes/mac/Data/poses/01.txt"

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
  
  string line;
  int i = 0;
  ifstream myfile (REALPOSE);
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}

Mat triangulate_Linear_LS(Mat mat_P_l, Mat mat_P_r, Mat warped_back_l, Mat warped_back_r)
{
    Mat A(4,3,CV_64FC1), b(4,1,CV_64FC1), X(3,1,CV_64FC1), X_homogeneous(4,1,CV_64FC1), W(1,1,CV_64FC1);
    W.at<double>(0,0) = 1.0;
    A.at<double>(0,0) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,0) - mat_P_l.at<double>(0,0);
    A.at<double>(0,1) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,1) - mat_P_l.at<double>(0,1);
    A.at<double>(0,2) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,2) - mat_P_l.at<double>(0,2);
    A.at<double>(1,0) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,0) - mat_P_l.at<double>(1,0);
    A.at<double>(1,1) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,1) - mat_P_l.at<double>(1,1);
    A.at<double>(1,2) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,2) - mat_P_l.at<double>(1,2);
    A.at<double>(2,0) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,0) - mat_P_r.at<double>(0,0);
    A.at<double>(2,1) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,1) - mat_P_r.at<double>(0,1);
    A.at<double>(2,2) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,2) - mat_P_r.at<double>(0,2);
    A.at<double>(3,0) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,0) - mat_P_r.at<double>(1,0);
    A.at<double>(3,1) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,1) - mat_P_r.at<double>(1,1);
    A.at<double>(3,2) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,2) - mat_P_r.at<double>(1,2);
    b.at<double>(0,0) = -((warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,3) - mat_P_l.at<double>(0,3));
    b.at<double>(1,0) = -((warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,3) - mat_P_l.at<double>(1,3));
    b.at<double>(2,0) = -((warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,3) - mat_P_r.at<double>(0,3));
    b.at<double>(3,0) = -((warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,3) - mat_P_r.at<double>(1,3));
    solve(A,b,X,DECOMP_SVD);
    vconcat(X,W,X_homogeneous);
    return X_homogeneous;
}



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

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
    return Point2d
    (
     (p.x-K.at<double>(0,2))/K.at<double>(0,0),
     (p.y-K.at<double>(1,2))/K.at<double>(1,1)
     );
}

inline double distance( double x, double y, double z)
{
    return sqrt( x * x + y * y + z * z);
}

void triangulation( const Point2d &pt1,const Point2d &pt2, const Mat &R, const Mat &t, Point3d &outpt)
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
    
    Mat Pj = (Mat_<double>(3,4) <<  1.8144486313396042e+03,0  ,2.0521093136805948e+03,0,
              0, 1.8144486313396042e+03, 1.0898609600712157e+03,0,
              0, 0, 1,0);
    
    Mat pt_4d;
    //Point2d pt1(30.60129500574, 114.40327060991);//, 22.064 };
    //Point2d pt2(30.60129672252, 114.40331230623);//, 22.095 };
    vector<Point2d> pts_1, pts_2;
    
#if 0
    pts_1.push_back(Point2d(114.40327060991,30.60129500574));
    pts_2.push_back(Point2d(114.40331230623,30.60129672252));
#else
    pts_1.push_back(pixel2cam(pt1, K));
    pts_2.push_back(pixel2cam(pt2, K));
    
//    pts_1.push_back(pt1);
//    pts_2.push_back(pt2);
#endif
    
//      triangulatePoints(T1, T2, pts_1, pts_2, pt_4d);
    
//    Mat ppt1 = (Mat_<double>(2,1)<< pt1.x,pt1.y);
//    Mat ppt2 = (Mat_<double>(2,1)<< pt2.x,pt2.y);
//    cout << ppt1 << endl << ppt2 << endl;
//
    Point2d tempt1 = pixel2cam(pt1,K);
    Point2d tempt2 = pixel2cam(pt2,K);
//    pt_4d = IterativeLinearLSTriangulation(Point3d(tempt1.x,tempt1.y,1),T1,Point3d(tempt2.x,tempt2.y,1),T2);
    pt_4d = LinearLSTriangulation(Point3d(tempt1.x,tempt1.y,1),T1,Point3d(tempt2.x,tempt2.y,1),T2);
    cout << pt_4d  << endl;
    Mat x = pt_4d.col(0);
    x = x/x.at<double>(3,0);
    cout << "x-w :" << x.at<double>(3,0) << endl;
    outpt = Point3d(x.at<double>(0,0),x.at<double>(1,0),x.at<double>(2,0));
    
    cout << "ouput pos :" << outpt.x  << " " << outpt.y << " " << outpt.z << endl;
}


int main( int argc, char** argv )	{

  Mat img_1, img_2;
  Mat R_f, t_f; //the final rotation and tranlation vectors containing the 
  Mat r_f;
    
  ofstream myfile;
  myfile.open ("results1_1.txt");

  char filename1[200];
  char filename2[200];
//  sprintf(filename1, "%s/image_0/%06d.png",IMAGEPATH, 0);
//  sprintf(filename2, "%s/image_0/%06d.png",IMAGEPATH, 1);

    sprintf(filename1, "/Volumes/mac/Data/measure/4014.jpg",IMAGEPATH, 0);
    sprintf(filename2, "/Volumes/mac/Data/measure/4015.jpg",IMAGEPATH, 1);

  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 50);

  //read the first two frames from the dataset
  Mat img_1_c = imread(filename1);
  Mat img_2_c = imread(filename2);

  if ( !img_1_c.data || !img_2_c.data ) { 
    std::cout<< " --(!) Error reading images " << std::endl; return -1;
  }

  // we work with grayscale images
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

  // feature detection, tracking
  vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
  featureDetection(img_1, points1);        //detect features in img_1
  vector<uchar> status;
  featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2

  //TODO: add a fucntion to load these values directly from KITTI's calib files
  // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
    
    
  double focal = 1.8144486313396042e+03;
  cv::Point2d pp(2.0521093136805948e+03, 1.0898609600712157e+03);
  //recovering the pose and the essential matrix
  Mat E, R, t, mask;
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R, t, focal, pp, mask);
//    cout << mask << endl;
    
    cout << R << endl;
    cout << t << endl;

    
//    double scale = GeoMath::ComputeDistance(114.40327060991,30.60129500574,
//                                            114.40331230623,30.60129672252);
//    double dt = scale / distance(t.at<double>(0,0), t.at<double>(1,0),t.at<double>(2,0));
    
    Point2d pt1(2735, 850);//, 22.064 };
    Point2d pt2(2880, 810);//, 22.095 };
    
    Point3d outpt;
    triangulation(pt1, pt2, R, t, outpt);
    double scale = GeoMath::ComputeDistance(114.40327060991,30.60129500574,
                                            114.40331230623,30.60129672252);
    double dt = scale / distance(t.at<double>(0,0), t.at<double>(1,0),t.at<double>(2,0));
    
    cout << outpt << " " << distance(outpt.x, outpt.y, outpt.z) * dt << endl;
  
  return 0;
}
