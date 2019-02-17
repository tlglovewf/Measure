//
//  main.cpp
//  FunctionTest
//
//  Created by TuLigen on 2019/1/22.
//

#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include "MathHelper.h"
using namespace std;
int main(int argc, const char * argv[]) {
    // insert code here...
    //
    double lon = GeoMath::tileX2Lon(215856, 18);
    double lat = GeoMath::tileY2Lat(99290, 18);
    
    
    cout << setprecision(15) << lat << "," << lon << endl;
    
    int x = GeoMath::Lon2tileX(108.837260567411, 18);
    int y = GeoMath::Lat2tileY(34.2000201873549, 18);
    cout << 18 << "/" << x << "/" << y << endl;
    char output[255] = {0};
   
    sprintf(output,"http://fs.navinfo.com/smapapi/v2x/Trafficsignal/%d/%d/%d",18,x,y);
    
    cout << output << endl;
    
//    "http://fs.navinfo.com/smapapi/smap-data/data/Railway/18/215856/99292?token=25cc55a69ea7422182d00d6b7c0ffa93&solu=716
//http://fs.navinfo.com/smapapi/v2x/LinkFace/18/215856/99292?token=25cc55a69ea7422182d00d6b7c0ffa93&solu=716
    return 0;
}
