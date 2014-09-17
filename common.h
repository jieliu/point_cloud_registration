#ifndef COMMON_H
#define COMMON_H
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cfloat>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef struct PointCloudShape
{
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
}PointCloudShape;

typedef struct LimitShape
{
    float beg, end;
}LimitShape;

//spatial system
typedef enum Quadrant { FirstQuadrant, SecondQuadrant, ThirdQuadrant, FourthQuadrant } Quadrant;
/**
 * @brief getQuadrant, used to transform input value to according quadrant
 * @param value
 * @return
 */
Quadrant getQuadrant(int value)
{
    switch(value)
    {
    case 1:
        return FirstQuadrant;
    case 2:
        return SecondQuadrant;
    case 3:
        return ThirdQuadrant;
    case 4:
        return FourthQuadrant;
    }
}
#endif // COMMON_H
