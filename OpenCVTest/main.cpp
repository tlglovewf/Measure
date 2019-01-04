//
//  main.cpp
//  OpenCvTest
//
//  Created by TuLigen on 2018/12/29.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

using namespace cv;
using namespace std;

#define  TEST1 0

#if TEST1
const Point2d input_pt (2735,850);
const Point2d output_pt(2880,810);



#else
const Point2d input_pt (2979.1,711.3);
const Point2d output_pt(3080.2,698.3);




#endif



typedef std::vector<KeyPoint>  KeyVector;
typedef std::vector<DMatch>    MatchVector;

const int boarder = 20;     // 边缘宽度
const int width   = 640;      // 宽度
const int height  = 480;      // 高度
const int ncc_window_size = 6;    // NCC 取的窗口半宽度
const int ncc_area = (2*ncc_window_size+1)*(2*ncc_window_size+1); // NCC窗口面积

// 检测一个点是否在图像边框内
inline bool inside( const Vector2d& pt ) {
    return pt(0,0) >= boarder && pt(1,0)>=boarder
    && pt(0,0)+boarder<width && pt(1,0)+boarder<=height;
}
// 双线性灰度插值
inline double getBilinearInterpolatedValue( const Mat& img, const Point2d& pt ) {
    uchar* d = & img.data[ int(pt.y)*img.step+int(pt.x) ];
    double xx = pt.x - floor(pt.x);
    double yy = pt.y - floor(pt.y);
    return  (( 1-xx ) * ( 1-yy ) * double(d[0]) +
             xx* ( 1-yy ) * double(d[1]) +
             ( 1-xx ) *yy* double(d[img.step]) +
             xx*yy*double(d[img.step+1]))/255.0;
}


class NCC_Match
{
public:
    NCC_Match(const Mat &mat,const Point2d &pt):_mat(mat)
    {
        // 零均值-归一化互相关
        // 先算均值
        for ( int x=-ncc_window_size; x<=ncc_window_size; x++ )
            for ( int y=-ncc_window_size; y<=ncc_window_size; y++ )
            {
                double value_ref = double(mat.ptr<uchar>( int(y+pt.y) )[ int(x+pt.x) ])/255.0;
                _mean += value_ref;
                
                _values.push_back(value_ref);
                
            }
        
        _mean /= ncc_area;
        
        // 计算 Zero mean NCC
        for ( int i=0; i<_values.size(); i++ )
        {
            _demoniator += (_values[i]-_mean)*(_values[i]-_mean);
        }
    }
    double ncc_value(const Mat &cur,const Point2d &pt)
    {
        // 零均值-归一化互相关
        // 先算均值
        double mean_ref = 0, mean_curr = 0;
        vector<double> values_curr; // 参考帧和当前帧的均值
        for ( int x=-ncc_window_size; x<=ncc_window_size; x++ )
            for ( int y=-ncc_window_size; y<=ncc_window_size; y++ )
            {
                double value_curr = getBilinearInterpolatedValue( cur, pt+Point2d(x,y) );
                mean_curr += value_curr;
                
                values_curr.push_back(value_curr);
            }
        
        mean_curr /= ncc_area;
        
        // 计算 Zero mean NCC
        double numerator = 0, cur_demoniator = 0;
        for ( int i=0; i < _values.size(); i++ )
        {
            double n = (_values[i]-mean_ref) * (values_curr[i]-mean_curr);
            numerator += n;
            cur_demoniator += (values_curr[i]-mean_curr)*(values_curr[i]-mean_curr);
        }
        return numerator / sqrt( _demoniator * cur_demoniator+1e-10 );   // 防止分母出现零
    }
protected:
    const Mat&      _mat;
    double          _mean;
    vector<double>  _values;
    double          _demoniator;
};


// 计算 NCC 评分
double NCC (
            const Mat& ref, const Mat& curr,
            const Point2d& pt_ref, const Point2d& pt_curr
            )
{
    // 零均值-归一化互相关
    // 先算均值
    double mean_ref = 0, mean_curr = 0;
    vector<double> values_ref, values_curr; // 参考帧和当前帧的均值
    for ( int x=-ncc_window_size; x<=ncc_window_size; x++ )
        for ( int y=-ncc_window_size; y<=ncc_window_size; y++ )
        {
            double value_ref = double(ref.ptr<uchar>( int(y+pt_ref.y) )[ int(x+pt_ref.x) ])/255.0;
            mean_ref += value_ref;
            
            double value_curr = getBilinearInterpolatedValue( curr, pt_curr+Point2d(x,y) );
            mean_curr += value_curr;
            
            values_ref.push_back(value_ref);
            values_curr.push_back(value_curr);
        }
    
    mean_ref /= ncc_area;
    mean_curr /= ncc_area;
    
    // 计算 Zero mean NCC
    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for ( int i=0; i<values_ref.size(); i++ )
    {
        double n = (values_ref[i]-mean_ref) * (values_curr[i]-mean_curr);
        numerator += n;
        demoniator1 += (values_ref[i]-mean_ref)*(values_ref[i]-mean_ref);
        demoniator2 += (values_curr[i]-mean_curr)*(values_curr[i]-mean_curr);
    }
    return numerator / sqrt( demoniator1*demoniator2+1e-10 );   // 防止分母出现零
}

inline double computeY(uint x, const cv::Vec<float, 3> &fn)
{
    assert( fabs(fn[1]) > 1e-6);
    return (-fn[2] - fn[0] * x) / fn[1];
}


//type == 1, 2, 3 分别对应取均值、调用opencv的API、运用转换公式
Mat turnIntoGray(Mat input, int type) {
    
    if (input.channels() == 1)return input;
    
    int row = input.rows;
    int col = input.cols;
    Mat output(row, col, CV_8UC1);
    
    if (type == 1 || type == 3) {
        for (int i = 0; i < row; i ++) {
            for (int j = 0; j < col; j ++) {
                double temp1 = input.at<Vec3b>(i, j)[0]; //B
                double temp2 = input.at<Vec3b>(i, j)[1]; //G
                double temp3 = input.at<Vec3b>(i, j)[2]; //R
                if (type == 1)output.ptr<uchar>(i)[j] = static_cast<uchar>((temp1 + temp2 + temp3) / 3);
                else output.ptr<uchar>(i)[j] = static_cast<uchar>(temp3 * 0.299 + temp2 * 0.587 + temp1 * 0.114);
            }
        }
    }
    else if (type == 2) {
        cvtColor(input, output, CV_BGR2GRAY);
    }
    return output;
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




int main(void)
{
    
#if TEST1
    Mat img1 =imread("/Volumes/mac/Data/measure/4014.jpg");
    Mat img2 =imread("/Volumes/mac/Data/measure/4015.jpg");
#else
    Mat img1 =imread("/Volumes/mac/Data/measure/1_1.jpg");
    Mat img2 =imread("/Volumes/mac/Data/measure/1_2.jpg");
#endif
    KeyVector img_key1, img_key2;
    Mat descriptor_1, descriptor_2;
    
    Mat img_out1 = turnIntoGray(img1, 2);
    Mat img_out2 = turnIntoGray(img2, 2);
    
    Mat img_match;

    MatchVector goodmatches;
 
    vector<Point2f> points1;
    vector<Point2f> points2;
#if 1
    Ptr<ORB> orb = ORB::create(500, 2.0f, 8 ,31, 0, 2, ORB::HARRIS_SCORE,31,20);

#define IMG1 img1
#define IMG2 img2
    orb->detect(IMG1, img_key1);
    orb->detect(IMG2, img_key2);
    
    orb->compute(IMG1, img_key1, descriptor_1);
    orb->compute(IMG2, img_key2, descriptor_2);
#undef IMG1
#undef IMG2
    
    BFMatcher matcher(NORM_HAMMING);
    MatchVector tmpmatches;
    matcher.match(descriptor_1, descriptor_2, tmpmatches);
    
    double min_dist = 10000, max_dist = 0;
    
    for(int i = 0; i < descriptor_1.rows;++i)
    {
        double dist = tmpmatches[i].distance;
        if(dist < min_dist)min_dist = dist;
        if(dist > max_dist)max_dist = dist;
    }
    
    cout << " mindist : " << min_dist << endl;
    cout << " maxdist : " << max_dist << endl;
    
    for(int i = 0; i < tmpmatches.size();++i)
    {
        const int sz = 5;
        Rect2f rect(0,0,sz,sz);
        if( (img_key1[tmpmatches[i].queryIdx].pt - img_key2[tmpmatches[i].trainIdx].pt).inside(rect))
            continue;
//                Rect2f  rect(0,1600,4096,600);
//                if(img_key1[tmpmatches[i].queryIdx].pt.inside(rect) )
//                    continue;
        
        if( tmpmatches[i].distance <= max(2 * min_dist,30.0) )
        {
            goodmatches.push_back(tmpmatches[i]);
        }
    }
    
    
#else
    vector<uchar> status;
    featureDetection(img_out1,points1);
    featureTracking(img_out1, img_out2, points1, points2, status);
    
#endif
    
    
    
    
    for(int i = 0;i < (int)tmpmatches.size(); ++i)
    {
        points1.push_back( img_key1[tmpmatches[i].queryIdx].pt);
        points2.push_back( img_key2[tmpmatches[i].trainIdx].pt);
    }
    
    Mat fdmat = findFundamentalMat(points1, points2);
    
    std::vector<cv::Vec<float, 3>>  epilines1, epilines2;
    
    points1.clear();
    
    points1.emplace_back(input_pt);
    
    //极线匹配      points1 img1中点像素坐标     fdmat基础矩阵   Each line \f$ax + by + c=0\f$ is encoded by 3 numbers \f$(a, b, c)\f$ .
    computeCorrespondEpilines(points1, 1, fdmat, epilines1);
//    computeCorrespondEpilines(points2, 2, fdmat, epilines2);
    
    cout << " 极线 : " << epilines1.size() << " " <<  epilines1[0][0] << " "
                                                <<    epilines1[0][1] << " "
                                                <<    epilines1[0][0] <<endl;
    
   
    
    for(uint i = 0; i < points1.size() ; ++i)
    {
        Scalar color = Scalar::all(-1);//随机产生颜色
        Scalar red = Scalar(0,0,255);//bgr
        circle(img1, input_pt, 5, Scalar(0,255,0));
        
        circle(img2, input_pt, 5, Scalar(0,255,0));
        circle(img2, output_pt , 5, red);//在img2中绘制出真实的同名点
         //绘制出极线    上面的验证点应该正好在绘制出的极线上
        double y1 = -epilines1[i][2] / epilines1[i][1];
        double y2 = -(epilines1[i][2] + epilines1[i][0] * img2.cols) / epilines1[i][1];
        line(img2, Point(0, y1), Point(img2.cols, y2), color,1);
        
        line(img1, Point(0, y1), Point(img2.cols, y2), color,1);
        cout << "极线 bg : " << y1 << " " << "ed : " << y2 << endl;
    }
    
    //块匹配
    
    const double searchlen = input_pt.x + 300;
    const double space = 0.7;
    
    double best_ncc = -1.0;
    Point2d best_px_curr ;

    NCC_Match ncc_match(img_out1,input_pt);
    
    for( double x = input_pt.x ; x < searchlen; x += space)
    {
        Point2d px_curr(x, computeY(x, epilines1[0]));
       
        double ncc= ncc_match.ncc_value(img_out2, px_curr);
        
        if(ncc > best_ncc)
        {
            best_ncc = ncc;
            best_px_curr = px_curr;
            
            circle(img2, px_curr, 1, Scalar(255,255,0));
            
        }
//        circle(img2, px_curr, 1, Scalar(255,0,0));
    }
    
    if( best_ncc < 0.70f)
    {
        throw  "error";
    }
    
    cout << "math pair : " << goodmatches.size() << endl;
    cout << " find the " << best_ncc << endl << output_pt.x << " " << output_pt.y << "  matching point is " << endl <<
    best_px_curr.x << " " << best_px_curr.y << endl;
    imwrite("/Volumes/mac/Data/measure/base.png", img1);
    
    drawMatches(img1, img_key1, img2, img_key2, goodmatches, img_match);
    
    imwrite("/Volumes/mac/Data/measure/search.png", img_match);
    
    waitKey(0);
    
    return 0;
}
