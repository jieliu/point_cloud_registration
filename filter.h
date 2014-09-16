#ifndef FILTER_H
#define FILTER_H
#include "common.h"
#include "adjust_parameter.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

/**
 * @brief cloudLimitFilter
 * @param cloudIn
 * @param cloudOut
 * @param limitBeg
 * @param limitEnd
 */
void
cloudLimitFilter (PointCloud::Ptr cloudIn, PointCloud::Ptr cloudOut, float limitBeg, float limitEnd)
{
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloudIn);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(limitBeg,limitEnd);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(limitBeg,limitEnd);
    pass.filter(*cloudOut);
}
/**
 * @brief cloudLimitFilter
 * @param cloudIn
 * @param cloudOut
 * @param limitBeg
 * @param limitEnd
 * @param role, value 0: x axis, 1: y axis, 2: both x,y axis useing the same limit value
 */
void
cloudLimitFilter (PointCloud::Ptr cloudIn, PointCloud::Ptr cloudOut, float limitBeg, float limitEnd, int role)
{
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloudIn);
    if( role == 1 )
    {
        pass.setFilterFieldName("y");
        pass.setFilterLimits(limitBeg,limitEnd);
    }
    else if( role == 0)
    {
        pass.setFilterFieldName("x");
        pass.setFilterLimits(limitBeg,limitEnd);
    }
    else if( role == 2)
    {
        pass.setFilterFieldName("y");
        pass.setFilterLimits(limitBeg,limitEnd);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(limitBeg,limitEnd);
    }
    else
    {
        fprintf(stdout, "role number is error!!\n");
    }

    pass.filter(*cloudOut);
}

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


/**
 * @brief cloudQuadrantFilter
 * @param cloudIn
 * @param cloudOut
 * @param shape
 * @param adjuestment
 * @param role
 * @param quadrant
 * @return LimitShape, x,y,z axis
 */
LimitShape
cloudQuadrantFilter (PointCloud::Ptr cloudIn, PointCloud::Ptr cloudOut, PointCloudShape shape, float adjuestment, int role, Quadrant quadrant)
{
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloudIn);

    assert( adjuestment >= 0 );
    float limitBeg = 0, limitEnd = 0;
    //first quadrant
    if( quadrant == FirstQuadrant )
    {
        if( role == 0 ) // x axis
        {
            limitBeg = 0 + adjuestment;
            limitEnd = shape.maxX - adjuestment;
        }
        else if( role == 1 ) // y axis
        {
            limitBeg = 0 + adjuestment;
            limitEnd = shape.maxY - adjuestment;
        }
    }
    else if( quadrant == SecondQuadrant )
    {
        if( role == 0 )
        {
            limitEnd = 0 - adjuestment;
            limitBeg = shape.minX + adjuestment;
        }
        else if( role == 1 )
        {
            limitBeg = 0 + adjuestment;
            limitEnd = shape.maxY - adjuestment;
        }
    }
    else if( quadrant == ThirdQuadrant )
    {
        if( role == 0 )
        {
            limitEnd = 0 - adjuestment;
            limitBeg = shape.minX + adjuestment;
        }
        else if( role == 1 )
        {
            limitEnd = 0 - adjuestment;
            limitBeg = shape.minY + adjuestment;
        }
    }
    else if( quadrant == FourthQuadrant )
    {
        if( role == 0 )
        {
            limitBeg = 0 + adjuestment;
            limitEnd = shape.maxX - adjuestment;
        }
        else if( role == 1 )
        {
            limitEnd = 0 - adjuestment;
            limitBeg = shape.minY + adjuestment;
        }
    }
    if( role == 0 )
    {
        pass.setFilterFieldName("x");
    }
    else if( role == 1 )
    {
        pass.setFilterFieldName("y");
    }
    pass.setFilterLimits(limitBeg, limitEnd);
    pass.filter(*cloudOut);
    LimitShape lshape;
    lshape.beg = limitBeg;
    lshape.end = limitEnd;
    return lshape;
    //cout << "line: " << __LINE__ << "\t" << "role: " << role << "[" << limitBeg << "," << limitEnd << "]" << endl;
}
/**
 * @brief voxelGridFilter
 * @param cloudIn
 * @param cloudOut
 */
void
voxelGridFilter(PointCloud::Ptr cloudIn, PointCloud::Ptr cloudOut)
{
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(0.1,0.1,0.1);
    grid.setInputCloud(cloudIn);
    grid.filter(*cloudOut);
}
/**
 * @brief statisticalOutlieFilter
 * @param cloud
 * @param cloudFilterd
 */
void
statisticalOutlieFilter(PointCloud::Ptr cloud, PointCloud::Ptr cloudFilterd)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloudFilterd);
}
#endif // FILTER_H
