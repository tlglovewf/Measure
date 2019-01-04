//
//  main.cpp
//  Test
//
//  Created by TuLigen on 2018/12/27.
//

#include "iostream"
#include "iomanip"
#include <opencv2/core/core.hpp>
#include "../MathHelper.h"
using namespace cv;



int main(void)
{
    cv::Point3d pt1 = GeoMath::ComputeXYZFromGPS(114.40327060991, 30.60129500574);
    cv::Point3d pt2 = GeoMath::ComputeXYZFromGPS(114.40331230623, 30.60129672252);
    
    cv::Point3d dir = pt2 - pt1;
    
    GeoMath::normalize(dir);
    
    cv::Point3d pt(1.72623,-0.607029,4.58698);
    double scale = GeoMath::ComputeDistance(114.40327060991,30.60129500574,114.40331230623,30.60129672252);
#if 0
    //根据行驶方向以及相对相机z轴夹角
    double θ = atan(pt.x / pt.z);

    cv::Point3d cdir(dir.x*cos(θ) + dir.y * sin(θ), -dir.y*cos(θ) + dir.x * sin(θ),0);
    GeoMath::normalize(cdir);
    double len = getLength(pt) * scale;
    pt2 =  pt2 + len * cdir;
    //114.40350395,30.60123760
    cv::Point2d gg= ComputeGPSFromXYZ(pt2);
    std::cout<< std::setprecision(10) << gg.x << "," << gg.y << " " <<
    GeoMath::ComputeDistance(gg.x,gg.y,114.40350395,30.60123760) << std::endl;
    
#else
    cv::Point3d zAixs = pt2 - pt1;
    GeoMath::normalize(zAixs);
    cv::Point3d yAixs = pt2;
    GeoMath::normalize(yAixs);
    Mat R = GeoMath::ComputeWorldTransMatrix(zAixs, yAixs, pt2);
    
    pt = pt * scale;
    
    std::cout << "++length : " << GeoMath::getLength(pt) << std::endl;
    Mat pos = (Mat_<double>(4,1) << pt.x,pt.y,pt.z,1) ;
    
    Mat rst = R * pos;
    
    cv::Point3d rstpt(rst.at<double>(0,0)/rst.at<double>(3,0),
                      rst.at<double>(1,0)/rst.at<double>(3,0),
                      rst.at<double>(2,0)/rst.at<double>(3,0));
    
    cv::Point2d  gp = GeoMath::ComputeGPSFromXYZ(rstpt);
    
    std::cout << std::setprecision(10) << scale << " " << gp.x << " " << gp.y << " " << GeoMath::ComputeDistance(gp.x, gp.y, 114.40350395, 30.60123760) <<  std::endl;
    
    std::cout << " +=======================+ " << std::endl;
    
    pos = (Mat_<double>(4,1) << 0,0,scale,1) ;
    
    yAixs = pt1;
    GeoMath::normalize(yAixs);
    R = GeoMath::ComputeWorldTransMatrix(zAixs, yAixs, pt1);
    
    rst = R * pos;
    
    cv::Point3d prept(rst.at<double>(0,0)/rst.at<double>(3,0),
                      rst.at<double>(1,0)/rst.at<double>(3,0),
                      rst.at<double>(2,0)/rst.at<double>(3,0));
    
    gp = GeoMath::ComputeGPSFromXYZ(prept);
    
    std::cout << std::setprecision(10) << scale << " " << gp.x << " " << gp.y << " " << GeoMath::ComputeDistance(gp.x, gp.y, 114.40331230623, 30.60129672252) <<  std::endl;
#endif
  
    return 0;
}
