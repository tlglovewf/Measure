//
//  GeoMath.h
//  Measure
//
//  Created by TuLigen on 2018/12/25.
//

#ifndef GeoMath_h
#define GeoMath_h

#include <opencv2/core/types.hpp>

const double PI64        = 3.1415926535897932384626433832795028841971693993751;
const double Half_PI64   = 1.5707963267948965579989817342720925807952880859375;
const double Earth_Radius= 6378137.0f;

#define D2R(X)   (X) * 0.01745329251994329547437168059786927187815308570862
#define R2D(X)   (X) / 0.01745329251994329547437168059786927187815308570862
#define FLATRATE  0.081819190782782381 // sqrt(EarthLongR * EarthLongR - EarthShortR * EarthShortR) / EarthLongR;
//gis数学
class GeoMath
{
public:
    
    /**  经纬度转墨卡托投影坐标
     @param lon 经度(degree）
     @param lat 维度
     @return    墨卡托投影坐标
    **/
    static cv::Point3d ComputeMerctorPosFromGPS( double lon, double lat)
    {
        cv::Point3d point;
        point.x = D2R(lon) * Earth_Radius  ;
        point.y = log(tan( D2R(lat) * 0.5 + 0.25 * PI64 )) * Earth_Radius;
        point.z = 0.0;
        return point;
    }
    
    /**  墨卡托投影坐标转经纬度
     @param gps 墨卡托投影坐标
     @return    经纬度pos(lon,lat)
     **/
    static cv::Point2d ComputeGPSFromMerctorPos( const cv::Point3d &gps)
    {
        cv::Point2d gPos;
        gPos.x = R2D(gps.x / Earth_Radius) ;
        gPos.y = R2D(2 * ( atan( exp(gps.y / Earth_Radius))) - Half_PI64);
        return gPos;
    }
    
    /**  计算距离
     @param gps 经纬度坐标
     @return    距离(m）
     **/
    static double ComputeDistance(double lng1, double lat1, double lng2, double lat2)
    {
        double radLat1 = D2R(lat1);
        double radLat2 = D2R(lat2);
        double a = radLat1 - radLat2;
        double b = D2R(lng1) - D2R(lng2);
        
        double s = 2 * asin(sqrt(pow(sin(a/2),2) +
                                 cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
        s = s * Earth_Radius;
        s = round(s * 10000) / 10000;
        return s;
    }
    
    /**
     *  单位化
     **/
    static cv::Point3d normalize( cv::Point3d &v)
    {
        double result;
        result = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        assert(0 != result);
        v = v / result;
        return v;
    }
    
    /**     获取向量长度
     * @param   v 向量
     & @return  长度
     */
    static double getLength(const cv::Point3d &v)
    {
        return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }
    
    /**  获取行驶方向
     @param prelon,prelat  前一帧经纬度
     @param curlon,curlat  当前帧经纬度
     @return    方向向量
     **/
    static cv::Point3d ComputeVehicleDirect(double prelon, double prelat, double curlon, double curlat)
    {
        cv::Point3d pre = ComputeMerctorPosFromGPS(prelon, prelat);
        cv::Point3d cur = ComputeMerctorPosFromGPS(curlon, curlat);
        
        cv::Point3d dir =  cur - pre;
        normalize(dir);
        return dir;
    }
    /** 像素坐标到像素物理坐标
     * @param   p 像素坐标
     * @param   K 内参矩阵
     & @return  像素物理坐标
     */
    static cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K)
    {
        return cv::Point2d
        (
         (p.x-K.at<double>(0,2))/K.at<double>(0,0),
         (p.y-K.at<double>(1,2))/K.at<double>(1,1)
         );
    }

    /** 获取相机->世界 转换矩阵
     * @param   zAxis eye direct  must normalize
     * @param   yAxis up direct   must normalize
     & @return  world transmatrix
     */
    static cv::Mat ComputeWorldTransMatrix(const cv::Point3d &zAxis, const cv::Point3d &yAxis,const cv::Point3d &pt)
    {
        cv::Point3d xAxis =  zAxis.cross(yAxis);
        
        GeoMath::normalize(xAxis);
        
        cv::Point3d uyAxis = zAxis.cross(xAxis);
        
        GeoMath::normalize(uyAxis);
        //构建世界变换矩阵  相机 -> 世界
        cv::Mat R = (cv::Mat_<double>(4,4) << xAxis.x , xAxis.y , xAxis.z ,pt.x,
                 uyAxis.x, uyAxis.y, uyAxis.z,pt.y,
                 zAxis.x , zAxis.y , zAxis.z ,pt.z,
                 0       ,0        ,0        ,1);
        return R;
    }
    
    /**     经纬度转空间坐标系
     * @param   lon 经度
     * @param   lat 维度
     & @return  空间坐标点
     */
    static cv::Point3d ComputeXYZFromGPS(double lon, double lat,double H1 = 0)
    {
        double a = Earth_Radius;
        double e = FLATRATE;//sqrt(a * a - b * b) / a;
        double N = a / sqrt(1 - e * e * sin(D2R(lat)) * sin(D2R(lat)));
        double WGS84_X = (N + H1) * cos(D2R(lat)) * cos(D2R(lon));
        double WGS84_Y = (N + H1) * cos(D2R(lat)) * sin(D2R(lon));
        double WGS84_Z = (N * (1 - (e * e)) + H1) * sin(D2R(lat));
        
        return cv::Point3d(WGS84_X,WGS84_Y,WGS84_Z);
    }
    
    /**     空间直角坐标系到经纬度坐标
     * @param   pt  空间坐标点
     & @return  经纬度
     */
    static cv::Point2d ComputeGPSFromXYZ(const cv::Point3d &pt)
    {
        double f, f1, f2;
        double p,zw, nnq;
        double b, l, h;
        
        double a = Earth_Radius;
        double eq = FLATRATE * FLATRATE;
        f = PI64 * 50 / 180;
        double x, y, z;
        x = pt.x;
        y = pt.y;
        z = pt.z;
        p = z / sqrt(x * x + y * y);
        do
        {
            zw = a / sqrt(1 - eq * sin(f) * sin(f));
            nnq = 1 - eq * zw / (sqrt(x * x + y * y) / cos(f));
            f1 = atan(p / nnq);
            f2 = f;
            f = f1;
        } while (!(abs(f2 - f1) < 10E-10));
        b = R2D(f);
        l = R2D( atan(y / x) );
        if (l < 0)
            l += 180.0;
        h = sqrt(x * x + y * y) / cos(f1) - a / sqrt(1 - eq * sin(f1) * sin(f1));
        return cv::Point2d{l, b};// h};
    }
};

#endif /* GeoMath_h */
